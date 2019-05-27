/**
 * Text To Speech synthesis library
 * Copyright (c) 2008 Clive Webster.  All rights reserved.
 *
 * Nov. 29th 2009 - Modified to work with Arduino by Gabriel Petrut:
 * The Text To Speech library uses Timer1 to generate the PWM
 * output on digital pin 10. The output signal needs to be fed
 * to an RC filter then through an amplifier to the speaker.
 * http://www.tehnorama.ro/minieric-modulul-de-control-si-sinteza-vocala/
 *
 * Modified to allow use of different PWM pins by Stephen Crane.
 * Modified for Timer5 on Arduino Mega2560 by Peter Dambrowsky.
 */

#include "tts.h"

#include "arduino_fixed.h"
#include <memory>
#include <cstring>
#include <cstdlib>
#include <algorithm>
#include <chrono>
#include <thread>


TTS::TTS() : AudioStream(0, nullptr), default_pitch_ { 4 }, task_running_ { true }, buffer_ {}, last_sample_ {} {
    const auto last { free_rtos_std::gthr_freertos::set_next_stacksize(TTS_TASK_STACK_SIZE) };
    task_handle_ = new std::thread([this]() { audio_processing(); });
    free_rtos_std::gthr_freertos::set_next_stacksize(last);

    free_rtos_std::gthr_freertos::set_priority(task_handle_, TTS_TASK_PRIORITY);
    free_rtos_std::gthr_freertos::set_name(task_handle_, "speak");
}

TTS::~TTS() {
    task_running_ = false;
    if (task_handle_ && task_handle_->joinable()) {
        task_handle_->join();
        delete task_handle_;
    }
}

bool TTS::speak(const std::string& text, const bool block) {
    auto p_text { new std::string(text) };
    if (block) {
        text_queue_.push(std::move(p_text));
        return true;
    } else {
        return text_queue_.try_push(std::move(p_text));
    }
}

bool TTS::is_playing() {
    return buffer_ || out_queue_.size();
}

void TTS::audio_processing() {
    std::string* p_element;
    while (task_running_) {
        text_queue_.pop(p_element);
        say_text(p_element->c_str());
        delete p_element;
    }
}

bool TTS::say_text(const char* original) {
    std::unique_ptr<char[]> buffer { new char[sizeof(phonemes_)] };
    if (!buffer) {
        return false;
    }

    bool res { false };

    if (text_to_phonemes(original, s_vocab, buffer.get())) {
        res = say_phonemes(buffer.get());
    }

    return res;
}

bool TTS::say_phonemes(const char* textp) {
    if (phonemes_to_data(textp, s_phonemes)) {
        // phonemes has list of sound bytes
        sound_on();

        // initialise random number seed
        srand(millis());

        // _630C
        int8_t byte1 = 0;
        int8_t punctuationPitchDelta = 0; // change in pitch due to fullstop or question mark

        // Q19
        for (uint8_t phonemeIn = 0 /* offset into text */, modifierIn = 0 /* offset into stuff in modifier */; phonemes_[phonemeIn];
             phonemeIn += 2, modifierIn += 2) {
            uint8_t byte2;
            uint8_t duration; // duration from text line
            uint8_t soundPos; // offset into sound data
            uint8_t fadeSpeed = 0;
            uint8_t sound1Num; // Sound data for the current phoneme
            uint8_t sound2Num; // Sound data for the next phoneme
            uint8_t sound2Stop; // Where the second sound should stop
            int8_t pitch1; // pitch for the first sound
            int8_t pitch2; // pitch for the second sound
            const SOUND_INDEX* soundIndex;

            char phoneme = phonemes_[phonemeIn];
            if (phoneme == 'z') {
                delay(15);
                continue;
            } else if (phoneme == '#') {
                continue;
            } else {
                // Collect info on sound 1
                soundIndex = &SoundIndex[phoneme - 'A'];
                sound1Num = soundIndex->SoundNumber;
                byte1 = soundIndex->byte1;
                byte2 = soundIndex->byte2;

                duration = phonemes_[phonemeIn + 1] - '0'; // Get duration from the input line
                if (duration != 1) {
                    duration <<= 1;
                }

                duration += 6; // scaled duration from the input line (at least 6)
                sound2Stop = 0x40 >> 1;

                pitch1 = modifier_[modifierIn];
                if (modifier_[modifierIn + 1] == 0 || pitch1 == -1) {
                    pitch1 = 10;
                    duration -= 6;
                } else if (modifier_[modifierIn + 1] == '0' || duration == 6) {
                    duration -= 6;
                }
                // q8
                pitch2 = modifier_[modifierIn + 2];
                if (modifier_[modifierIn + 3] == 0 || pitch2 == -1)
                    pitch2 = 10;

                // q10
                if (byte1 < 0) {
                    sound1Num = 0;
                    rand();
                    sound2Stop = (0x40 >> 1) + 2;
                } else {
                    // is positive
                    if (byte1 == 2) {
                        // 64A4
                        // Make a white noise sound!
                        uint8_t volume = (duration == 6) ? 15 : 1; // volume mask
                        for (duration <<= 2; duration > 0; duration--) {
                            play_tone(sound1Num, rand() % 0x3fu, 8, 12, 11, volume);
                            // Increase the volume
                            if (++volume == 16) {
                                volume = 15; // full volume from now on
                            }
                        }
                        continue;

                    } else {
                        // q11
                        if (byte1) {
                            delay(25);
                        }
                    }
                }
            }

            // 6186
            pitch1 += default_pitch_ + punctuationPitchDelta;
            if (pitch1 < 1) {
                pitch1 = 1;
            }

            pitch2 += default_pitch_ + punctuationPitchDelta;
            if (pitch2 < 1) {
                pitch2 = 1;
            }

            // get next phoneme
            phoneme = phonemes_[phonemeIn + 2];

            if (phoneme == 0 || phoneme == 'z') {
                if (duration == 1) {
                    delay(60);
                }
                phoneme = 'a'; // change to a pause
            } else {
                // s6
                if (byte2 != 1) {
                    byte2 = (byte2 + SoundIndex[phoneme - 'A'].byte2) >> 1;
                }

                if (byte1 < 0 || SoundIndex[phoneme - 'A'].byte1) {
                    phoneme = 'a'; // change to a pause
                }
            }

            // S10
            sound2Num = SoundIndex[phoneme - 'A'].SoundNumber;

            uint8_t sound1Duration = 0x80; // play half of sound 1
            if (sound2Num == sound1Num) {
                byte2 = duration;
            }

            // S11
            if ((byte2 >> 1) == 0) {
                sound1Duration = 0xff; // play all of sound 1
            } else {
                // The fade speed between the two sounds
                fadeSpeed = (sound1Duration + (byte2 >> 1)) / byte2;

                if (duration == 1) {
                    sound2Stop = 0x40; // dont play sound2
                    sound1Duration = 0xff; // play all of sound 1
                    pitch1 = 12;
                }
            }

            soundPos = 0;
            do {
                const uint8_t sound1Stop = (sound1Duration >> 2) & 0x3fu;
                const uint8_t sound1End = std::min(sound1Stop, sound2Stop);

                if (sound1Stop) {
                    soundPos = play_tone(sound1Num, soundPos, pitch1, pitch1, sound1End, 15);
                }

                // s18
                if (sound2Stop != 0x40) {
                    soundPos = play_tone(sound2Num, soundPos, pitch2, pitch2, sound2Stop - sound1End, 15);
                }
                // s23
                if (sound1Duration != 0xff && duration < byte2) {
                    // Fade sound1 out
                    sound1Duration -= fadeSpeed;
                    if (sound1Duration >= 0xC8) {
                        sound1Duration = 0; // stop playing sound 1
                    }
                }
                // Call any additional sound
                if (byte1 == -1) {
                    play(3, 30); // make an 'f' sound
                } else if (byte1 == -2) {
                    play(3, 29); // make an 's' sound
                } else if (byte1 == -3) {
                    play(3, 33); // make a 'th' sound
                } else if (byte1 == -4) {
                    play(3, 27); // make a 'sh' sound
                }

            } while (--duration);

            // Scan ahead to find a '.' or a '?' as this will change the pitch
            punctuationPitchDelta = 0;
            for (size_t i = 6; i > 0; i--) {
                const char next = phonemes_[phonemeIn + (i * 2)];
                if (next == 'i') {
                    // found a full stop
                    punctuationPitchDelta = 6 - i; // Lower the pitch
                } else if (next == 'h') {
                    // found a question mark
                    punctuationPitchDelta = i - 6; // Raise the pitch
                }
            }

            if (byte1 == 1) {
                delay(25);
            }
        } // next phoneme
        sound_off();
        return true;
    }

    return false;
}

/**
 * Enter:
 * src => English text in upper case
 * vocab => VOCAB array
 * dest => address to return result
 * return true if ok, or false if error
 */
bool TTS::text_to_phonemes(const char* src, const VOCAB* vocab, char* dest) const {
    int outIndex = 0; // Current offset into dest
    int inIndex = -1; // Starts at -1 so that a leading space is assumed

    while (inIndex == -1 || src[inIndex]) { // until end of text
        int maxMatch = 0; // Max chars matched on input text
        int numOut = 0; // Number of characters copied to output stream for the best match
        boolean endsInWhiteSpace = false;
        int maxWildcardPos = 0;

        // Get next phoneme, P2
        for (unsigned int ph = 0; ph < sizeof(s_vocab) / sizeof(s_vocab[0]); ph++) {
            int y, x;
            char wildcard = 0; // modifier
            int wildcardInPos = 0;
            boolean hasWhiteSpace = false;
            const char* text = vocab[ph].txt;
            const char* phon = vocab[ph].phoneme;

            for (y = 0;; y++) {
                char nextVocabChar = text[y];
                char nextCharIn = (y + inIndex == -1) ? ' ' : src[y + inIndex];
                if (nextCharIn >= 'a' && nextCharIn <= 'z') {
                    nextCharIn = nextCharIn - 'a' + 'A';
                }

                if (nextVocabChar == '#' && nextCharIn >= 'A' && nextCharIn <= 'Z') {
                    wildcard = nextCharIn; // The character equivalent to the '#'
                    wildcardInPos = y;
                    continue;
                }

                if (nextVocabChar == '_') {
                    // try to match against a white space
                    hasWhiteSpace = true;
                    if (is_whitespace(nextCharIn)) {
                        continue;
                    }
                    y--;
                    break;
                }
                // check for end of either string
                if (nextVocabChar == 0 || nextCharIn == 0) {
                    break;
                }

                if (nextVocabChar != nextCharIn) {
                    break;
                }
            }

            // See if it's the longest complete match so far
            if (y <= maxMatch || text[y]) {
                continue;
            }

            // This is the longest complete match
            maxMatch = y;
            maxWildcardPos = 0;
            x = outIndex; // offset into phoneme return data

            // Copy the matching phrase changing any '#' to the phoneme for the wildcard
            for (y = 0;; y++) {
                char c = phon[y];
                if (c == 0) {
                    break;
                }
                if (c == '#') {
                    if (phon[y + 1] == 0) {
                        // replacement ends in wildcard
                        maxWildcardPos = wildcardInPos;
                    } else {
                        x = copy_token(wildcard, dest, x, vocab); // Copy the phonemes for the wildcard character
                    }
                } else {
                    dest[x++] = c;
                }
            }
            dest[x] = 0;
            endsInWhiteSpace = hasWhiteSpace;

            // 14
            numOut = x - outIndex; // The number of bytes added
        }
        // 15 - end of vocab table

        // 16
        if (endsInWhiteSpace) {
            maxMatch--;
        }

        // 17
        if (maxMatch == 0) {
            // loggerP(PSTR("Mistake in SAY, no token for "));
            // logger(&src[inIndex]);
            // loggerCRLF();
            return false;
        }
        // 20
        outIndex += numOut;
        if (outIndex > 128 - 16) {
            // loggerP(PSTR("Mistake in SAY, text too long\n"));
            return false;
        }
        // 21
        inIndex += (maxWildcardPos > 0) ? maxWildcardPos : maxMatch;
    }
    return true;
}

/**
 * Convert phonemes to data string
 * Enter: textp = phonemes string
 * Return: phonemes = string of sound data
 *	   modifier = 2 bytes per sound data
 */
bool TTS::phonemes_to_data(const char* textp, const PHONEME* phoneme) {
    unsigned int phonemeOut = 0; // offset into the phonemes array
    unsigned int modifierOut = 0; // offset into the modifiers array
    unsigned int L81 = 0; // attenuate
    unsigned int L80 = 16;

    while (*textp) {
        // P20: Get next phoneme
        bool anyMatch = false;
        int longestMatch = 0;
        int numOut = 0; // The number of bytes copied to the output for the longest match

        // Get next phoneme, P2
        for (unsigned int ph = 0; ph < sizeof(s_phonemes) / sizeof(s_phonemes[0]); ph++) {
            int numChars;

            // Locate start of next phoneme
            const char* ph_text = phoneme[ph].txt;

            // Set 'numChars' to the number of characters
            // that we match against this phoneme
            for (numChars = 0; textp[numChars]; numChars++) {
                // get next input character and make lower case
                char nextChar = textp[numChars];
                if (nextChar >= 'A' && nextChar <= 'Z') {
                    nextChar = nextChar - 'A' + 'a';
                }

                if (nextChar != ph_text[numChars]) {
                    break;
                }
            }

            // if not the longest match so far then ignore
            if (numChars <= longestMatch) {
                continue;
            }

            // partial phoneme match
            if (ph_text[numChars]) {
                continue;
            }

            // P7: we have matched the whole phoneme
            longestMatch = numChars;

            // Copy phoneme data to 'phonemes'
            const char* ph_ph = phoneme[ph].phoneme;
            for (numOut = 0; ph_ph[numOut]; numOut++) {
                phonemes_[phonemeOut + numOut] = ph_ph[numOut];
            }

            L81 = phoneme[ph].attenuate + '0';
            anyMatch = true; // phoneme match found

            modifier_[modifierOut] = -1;
            modifier_[modifierOut + 1] = 0;

            // Get char from text after the phoneme and test if it is a numeric
            if (textp[longestMatch] >= '0' && textp[longestMatch] <= '9') {
                // Pitch change requested
                modifier_[modifierOut] = pitches_[textp[longestMatch] - '1'];
                modifier_[modifierOut + 1] = L81;
                longestMatch++;
            }
            // P10
            if (L81 != '0' && L81 != L80 && modifier_[modifierOut] >= 0) {
                modifier_[modifierOut - 2] = modifier_[modifierOut];
                modifier_[modifierOut - 1] = '0';
                continue;
            }
            // P11
            if ((textp[longestMatch - 1] | 0x20) == 0x20) {
                // end of input string or a space
                modifier_[modifierOut] = (modifierOut == 0) ? 16 : modifier_[modifierOut - 2];
            }
        } // next phoneme

        // p13
        L80 = L81;
        if (longestMatch == 0 && !anyMatch) {
            return false;
        }

        // Move over the bytes we have copied to the output
        phonemeOut += numOut;

        if (phonemeOut > sizeof(phonemes_) - 16) {
            return false;
        }

        // P16

        // Copy the modifier setting to each sound data element for this phoneme
        if (numOut > 2) {
            for (int count = 0; count != numOut; count += 2) {
                modifier_[modifierOut + count + 2] = modifier_[modifierOut + count];
                modifier_[modifierOut + count + 3] = 0;
            }
        }
        modifierOut += numOut;

        // p21
        textp += longestMatch;
    }

    phonemes_[phonemeOut++] = 'z';
    phonemes_[phonemeOut++] = 'z';
    phonemes_[phonemeOut++] = 'z';
    phonemes_[phonemeOut++] = 'z';

    while (phonemeOut < sizeof(phonemes_)) {
        phonemes_[phonemeOut++] = 0;
    }

    while (modifierOut < sizeof(modifier_)) {
        modifier_[modifierOut++] = -1;
        modifier_[modifierOut++] = 0;
    }

    return true;
}

/**
 * Find the single character 'token' in 'vocab' and append its phonemes to dest[x]
 */
size_t TTS::copy_token(char token, char* dest, size_t x, const VOCAB* vocab) const {
    for (size_t ph = 0; ph < sizeof(s_vocab) / sizeof(s_vocab[0]); ph++) {
        const char* txt = vocab[ph].txt;
        if (txt[0] == token && txt[1] == 0) {
            const char* src = vocab[ph].phoneme;
            while (*src) {
                dest[x++] = *src;
                src++;
            }
            break;
        }
    }
    return x;
}

void TTS::play(uint8_t duration, uint8_t soundNumber) {
    while (duration--) {
        play_tone(soundNumber, rand() % 0x3fu, 7, 7, 10, 15);
    }
}

uint8_t TTS::play_tone(uint8_t soundNum, uint8_t soundPos, uint8_t pitch1, uint8_t pitch2, uint8_t count, uint8_t volume) {
    const uint8_t* soundData = &SoundData[soundNum * 0x40];
    while (count-- > 0) {
        const uint8_t s = soundData[soundPos & 0x3fu];
        sound(s & volume, false);
        pause(pitch1);
        sound((s >> 4) & volume, false);
        pause(pitch2);

        soundPos++;
    }
    return soundPos & 0x3fu;
}

void TTS::sound_on() {
    last_sample_ = 0;
}

void TTS::sound_off() {
    if (buffer_ && buffer_idx_ < AUDIO_BLOCK_SAMPLES) {
        const size_t diff { AUDIO_BLOCK_SAMPLES - buffer_idx_ };
        for (size_t i { 0 }; i < diff; ++i) {
            sound(last_sample_, true);
        }
    }
}

void TTS::interpolate(const int16_t last_sample, const int16_t new_sample, const uint32_t steps, const uint32_t scaling) {
    const uint32_t stepsize { scaling / steps };
    for (uint32_t i { 1 }; i <= steps; ++i) {
        prepare_buffer();
        buffer_->data[buffer_idx_++] = static_cast<int16_t>((last_sample * (steps - i) + new_sample * i) * stepsize - 32768);
        commit_buffer();
    }
}

void TTS::sound(const uint8_t sample, const bool single) {
    const int16_t value { static_cast<int16_t>(sample) };
    interpolate(last_sample_, value, single ? 1 : SAMPLING_FREQ / 11025U, 128U * 16U);
    last_sample_ = value;
}

void TTS::pause(const uint8_t d) {
    const float us { d * 1.5f };
    const size_t samples { static_cast<size_t>(us / (1000000.f / static_cast<float>(SAMPLING_FREQ))) };
    for (size_t i { 0 }; i < samples; ++i) {
        sound(last_sample_, true);
    }
}

void TTS::delay(const uint8_t d) {
    const float us { d * 3127.f };
    const size_t samples { static_cast<size_t>(us / (1000000.f / static_cast<float>(SAMPLING_FREQ))) };
    for (size_t i { 0 }; i < samples; ++i) {
        sound(last_sample_, true);
    }
}

void TTS::prepare_buffer() {
    if (buffer_) {
        return;
    }

    buffer_ = allocate();
    while (!buffer_) {
        using namespace std::chrono_literals;
        std::this_thread::sleep_for(3ms);
        buffer_ = allocate();
    }
    buffer_idx_ = 0;
}

void TTS::commit_buffer() {
    if (buffer_idx_ == AUDIO_BLOCK_SAMPLES) {
        out_queue_.push(buffer_);
        buffer_ = nullptr;
    }
}

void TTS::update() {
    audio_block_t* p_element;
    if (out_queue_.try_pop(p_element)) {
        transmit(p_element);
        release(p_element);
    }
}
