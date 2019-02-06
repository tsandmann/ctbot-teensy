#!/bin/sh
ln -s doc/uml/_all.puml .
plantuml -tpdf -progress _all.puml
rm _all.puml
mkdir doc/out
mv ctbot-teensy.pdf doc/out/ 

ln -s doc/uml/_all_public.puml .
plantuml -tpdf -progress _all_public.puml
rm _all_public.puml
mv ctbot-teensy_public.pdf doc/out/ 

