import streamlit as st
import random
import string


ksayisi = st.slider("Şifre uzunluğu", 1, 15)
khsecisaret = st.checkbox("abc")
bhsecisaret = st.checkbox("ABC")
dsecisaret = st.checkbox("123")
spsecisaret = st.checkbox("-*/")


karakterler = ""

if khsecisaret:
    karakterler += string.ascii_lowercase
if bhsecisaret:
    karakterler += string.ascii_uppercase
if dsecisaret:
    karakterler += string.digits
if spsecisaret:
    karakterler += string.punctuation


if karakterler:
    sifre = "".join(random.choices(karakterler, k=ksayisi))
    st.write("Oluşturulan Şifre:", sifre)
else:
    st.warning("Lütfen en az bir karakter türü seçin.")
