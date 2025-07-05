#pip install streamlit
import streamlit as st
kdv=float(1.20)
bayi=float(1.15)
sonfiyat=float(0)
fiyat=(st.number_input("Aracın fiyatını giriniz"))
motorhacmi=(st.number_input("Motor hacmini giriniz"))
kwdeger=(st.number_input("Elektrikli ise Kw değerini giriniz"))
tip = st.radio(
    "Araç elektrikli mi?",
    ["Elektrikli", "İçten yanmalı"])
if st.button("Fiyatı Hesapla"):
    if tip=="İçten yanmalı":
         if motorhacmi < 1600:
            if fiyat < 184000:
                sonfiyat = (((fiyat * 1.45) * kdv) * bayi)
            elif 184000< fiyat > 220000:
                sonfiyat = (((fiyat * 1.50) * kdv) * bayi)
            elif 220000 < fiyat > 250000:
                sonfiyat = (((fiyat * 1.60) * kdv) * bayi)
            elif 250000<fiyat > 280000:
                sonfiyat = (((fiyat * 1.70) * kdv) * bayi)
            else:
                sonfiyat = (((fiyat * 1.80) * kdv) * bayi)
         if 1600 < motorhacmi < 2000:
            if fiyat < 170000:
              sonfiyat = (((fiyat * 2.3) * kdv) * bayi)
            else:
                sonfiyat = (((fiyat * 2.5) * kdv) * bayi)
         if motorhacmi > 2000 :
           sonfiyat = (((fiyat * 3.2) * kdv) * bayi)
    if tip=="Elektrikli":
        if kwdeger<160:
            if fiyat<700000:
                sonfiyat = (((fiyat * 1.1) * kdv) * bayi)
            else:
                sonfiyat = (((fiyat * 1.4) * kdv) * bayi)
        if kwdeger>160:
            if fiyat < 750000:
                sonfiyat = (((fiyat * 1.5) * kdv) * bayi)
            else:
                sonfiyat = (((fiyat * 1.6) * kdv) * bayi)


st.success(f"Son fiyat: {sonfiyat:,.0f} TL".replace(",", "."))






