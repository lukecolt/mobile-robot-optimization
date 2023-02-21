from asyncio.windows_events import NULL
import fnmatch
from importlib.resources import path
import math
import random
import csv
import copy
import tkinter as tk
import PIL
from PIL import Image, ImageDraw
from click import command
import time
import os
import glob
from matplotlib.font_manager import findSystemFonts
import optimalization as plik_mapy

algorytm_1_czas = 0
algorytm_2_czas = 0
#ZMIENNE GLOBALNE
WYSOKOSC_OKNA = 600
SZEROKOSC_OKNA = 600

#TWORZENIE OKNA
okno = tk.Tk()
okno.title("Program optymalizacji ruchu robota")
okno.minsize(height=str(WYSOKOSC_OKNA), width=str(SZEROKOSC_OKNA))
okno.maxsize(height=str(WYSOKOSC_OKNA), width=str(SZEROKOSC_OKNA))
pathobecny = os.path.realpath(os.path.join(os.getcwd(), os.path.dirname(__file__))) + "\\" + "mapy"


def menu_glowne():

    def okno_menu_algorytmy():

            
        def przeglad_map(okno_stare):
            '''
            Funkcja służąca do przeglądania stworzonych już map

            Funkcja przyjmuje zmienną typu Frame ("okno_stare"),
            które jest oknem, które ma zostać aktywowane w momencie powrotu do menu mapy

            Funkcja nie zwraca żadnej wartości!!!!
            '''
            def pobierz_mapy(): 
                '''
                Funkcja służąca pobraniu spisu wygeneroowany map .ppm

                Funkcja nie zwraca żadnej wartości
                ''' 
                sciezka = pathobecny
                #sciezka = "C:\\Users\\User\\OneDrive - Politechnika Wroclawska\\ISA Semestr1\\AO-projekt\\projektpodlogamapy"
                lista = fnmatch.filter(os.listdir(sciezka), "*.ppm")
                for i in range(0, len(lista)):
                    zbior_dostepnych_map.insert('end', str(lista[i]))

            def podglad_mapy():
                '''
                Funkcja służąca do wygenerowania mapy wybranej z listy w oknie

                Funkcja nie zwraca żadnej wartości
                '''
                global mapa_png
                global okno_mapa
                global okno_ramka
                global mapa

                if czy2['wygenerowana_mapa2'] == True:
                    okno_mapa.delete('all')
                    okno_ramka.delete('all')
                    czy2['wygenerowana_mapa2'] = False

                if zbior_dostepnych_map.curselection() != ():
                    nr_mapy = zbior_dostepnych_map.curselection()
                    nazwa_ppm = zbior_dostepnych_map.get(nr_mapy)
                    sciezka_ppm = pathobecny + '\\' + nazwa_ppm
                    #sciezka_ppm = "C:\\Users\\User\\OneDrive - Politechnika Wroclawska\\ISA Semestr1\\AO-projekt\\projektpodlogamapy" + '\\' + nazwa_ppm
                    nazwa_png = nazwa_ppm.replace(".ppm", ".png")
                    sciezka_png = pathobecny + '\\' + nazwa_png
                    #sciezka_png = "C:\\Users\\User\\OneDrive - Politechnika Wroclawska\\ISA Semestr1\\AO-projekt\\projektpodlogamapy" + '\\' + nazwa_png
                    mapa = plik_mapy.odczyt_mapy_z_ppm(sciezka_ppm)
                    mapa_png = tk.PhotoImage(file=sciezka_png)
                    okno_ramka = tk.Canvas(okno_przegladu_mapy, height=str(int(mapa['wysokosc'])+10), width=str(int(mapa['szerokosc'])+10), highlightthickness=0)
                    okno_ramka.create_rectangle(0, 0, 800+10 ,800+10, fill="black", width=0)
                    okno_ramka.pack()
                    okno_ramka.place(x=405, y=405, anchor="center")
                    okno_mapa = tk.Canvas(okno_przegladu_mapy, height=mapa['wysokosc'], width=mapa['szerokosc'], highlightthickness=0)
                    okno_mapa.create_image(0, 0, anchor="nw", image=mapa_png, tag='mapa')
                    okno_mapa.pack()
                    okno_mapa.place(x=405, y=405, anchor="center")
                    czy2['wygenerowana_mapa2'] = True

            def wybor_mapy():
                if zbior_dostepnych_map.curselection() != ():
                    nr_mapy = zbior_dostepnych_map.curselection()
                    nazwa_ppm = zbior_dostepnych_map.get(nr_mapy)

                powrot_menu_algorytmy(okno_przegladu_mapy, okno_stare, True, nazwa_ppm)

            def powrot_menu_algorytmy(okno_aktualne, okno_nowe, flaga_wybor, nazwa):
                global mapa_png
                global okno_ma
                global okno_ram
                global sciezka_png
                okno_aktualne.pack_forget()
                okno_nowe.pack()
                print(czy3['wygenerowana_mapa3'])
                if czy3['wygenerowana_mapa3'] == True:
                    okno_ma.delete('all')
                    okno_ram.delete('all')
                    czy3['wygenerowana_mapa3'] = False              
                
                if flaga_wybor == True:
                    nazwa_ppm = nazwa
                    sciezka_ppm = pathobecny + '\\' + nazwa_ppm
                    #sciezka_ppm = "C:\\Users\\User\\OneDrive - Politechnika Wroclawska\\ISA Semestr1\\AO-projekt\\projektpodlogamapy" + '\\' + nazwa_ppm
                    nazwa_png = nazwa_ppm.replace(".ppm", ".png")
                    sciezka_png = pathobecny + '\\' + nazwa_png
                    #sciezka_png = "C:\\Users\\User\\OneDrive - Politechnika Wroclawska\\ISA Semestr1\\AO-projekt\\projektpodlogamapy" + '\\' + nazwa_png
                    nazwa_csv= nazwa_ppm.replace(".ppm", ".csv")
                    sciezka_csv = pathobecny + '\\' + nazwa_csv
                    #sciezka_csv = "C:\\Users\\User\\OneDrive - Politechnika Wroclawska\\ISA Semestr1\\AO-projekt\\projektpodlogamapy" + '\\' + nazwa_csv
                    mapa = plik_mapy.odczyt_mapy_z_ppm(sciezka_ppm)
                    plik_mapy.odczytaj_mape_z_csv(sciezka_csv)
                    mapa_png = tk.PhotoImage(file=sciezka_png)
                    okno_ram = tk.Canvas(okno_nowe, height=str(int(mapa['wysokosc'])+10), width=str(int(mapa['szerokosc'])+10), highlightthickness=0)
                    okno_ram.create_rectangle(0, 0, 800+10 ,800+10, fill="black", width=0)
                    okno_ram.pack()
                    okno_ram.place(x=405, y=405, anchor="center")
                    okno_ma = tk.Canvas(okno_nowe, height=str(int(mapa['wysokosc'])), width=str(int(mapa['szerokosc'])), highlightthickness=0)
                    okno_ma.create_image(0, 0, anchor="nw", image=mapa_png, tag='mapa')
                    okno_ma.pack()
                    okno_ma.place(x=405, y=405, anchor="center")
                    czy3['wygenerowana_mapa3'] = True
                    print(czy3['wygenerowana_mapa3'])

            #ZMIENNE
            okno_stare.pack_forget()
            okno_przegladu_mapy = tk.Frame(okno_menu_algorytmy,  height=str(810), width=str(1000))
            
            czy2 = {'wygenerowana_mapa2': False}
        
            #TWORZENIE ELEMENTÓW MAPY
            tekst_dostepne_mapy = tk.Label(okno_przegladu_mapy, text="Lista dostępnych map:", height=str(0), width=str(0), font = ("timesnewroman", 12))
            rolka_pionowa = tk.Scrollbar(okno_przegladu_mapy, orient='vertical')
            rolka_pozioma = tk.Scrollbar(okno_przegladu_mapy, orient='horizontal')
            zbior_dostepnych_map = tk.Listbox(okno_przegladu_mapy, height=str(35), width=str(24), font = ("timesnewroman", 10), yscrollcommand=rolka_pionowa.set, xscrollcommand=rolka_pozioma.set)

            przycisk_podglądaj_mape = tk.Button(okno_przegladu_mapy, text="Podgląd mapy", width='15', font = ("timesnewroman", 15), command=lambda: podglad_mapy())
            przycisk_wybor_mapy = tk.Button(okno_przegladu_mapy, text="Wybór mapy", width='15', font = ("timesnewroman", 15), command=lambda: wybor_mapy())
            przycisk_wyjscie = tk.Button(okno_przegladu_mapy, text="Powrót do menu", width='15', font = ("timesnewroman", 15), command=lambda: powrot_menu_algorytmy(okno_przegladu_mapy, okno_stare, False, NULL))
        
            pobierz_mapy()
            
            rolka_pionowa.config(command = zbior_dostepnych_map.yview)
            rolka_pozioma.config(command = zbior_dostepnych_map.xview)
        
            tekst_dostepne_mapy.pack()
            tekst_dostepne_mapy.place(x=810, y=0, anchor="nw")
            zbior_dostepnych_map.pack()
            zbior_dostepnych_map.place(x=810, y=25, anchor="nw")
            rolka_pionowa.pack()
            rolka_pionowa.place(x=1000, y=25, height=600, anchor="ne")
            rolka_pozioma.pack()
            rolka_pozioma.place(x=810, y=630, width=171, anchor="nw")
        
            przycisk_podglądaj_mape.pack()
            przycisk_podglądaj_mape.place(x=905, y=680, anchor="center")
            przycisk_wybor_mapy.pack()
            przycisk_wybor_mapy.place(x=905, y=730, anchor="center")
            przycisk_wyjscie.pack()
            przycisk_wyjscie.place(x=905, y=780, anchor="center")
        
            okno_przegladu_mapy.pack()

        def aktualizuj_wyswietlany_czas():
            global algorytm_1_czas 
            global algorytm_2_czas
            global roznica 

            algorytm_1_czas = round(algorytm_1_czas, 3)
            algorytm_2_czas = round(algorytm_2_czas, 3)
            roznica = algorytm_1_czas - algorytm_2_czas # CZASY OBYDWU SĄ WRZUCONE JAKO GLOBALNE ZMIENNE!!! 
            roznica = round(roznica, 3)
            roznica = tk.Message(okno_menu_algorytmy, width='350', font = ("timesnewroman", 12, 'bold'), text=("Różnica czasu: " + str(roznica)))
            czas_al1 = tk.Message(okno_menu_algorytmy, width='350', font = ("timesnewroman", 12, 'bold'), text=("Czas Algorytmu LPA: " + str(algorytm_2_czas)))
            czas_al2 = tk.Message(okno_menu_algorytmy, width='350', font = ("timesnewroman", 12, 'bold'), text=("Czas Algorytmu DLite: " + str(algorytm_1_czas)))
            czas_al1.pack()
            czas_al1.place(x=700, y=45, anchor="nw")
            czas_al2.pack()
            czas_al2.place(x=700, y=75, anchor="nw")
            roznica.pack()
            roznica.place(x=700, y=105, anchor="nw") 

        def dstarlite():
            global algorytm_1_czas
            algorytm_1_czas = plik_mapy.animacja_dstar(sciezka_png, 1)
            aktualizuj_wyswietlany_czas()
        
        def astar():
            global algorytm_2_czas
            algorytm_2_czas = plik_mapy.animacja_dstar(sciezka_png, 0)
            aktualizuj_wyswietlany_czas()

        def wyczysc(): 
            global algorytm_1_czas
            global algorytm_2_czas
            global roznica
            algorytm_1_czas = 0
            algorytm_2_czas = 0
            roznica = 0
            if czy3['wygenerowana_mapa3'] == True:
                    okno_ma.delete('all')
                    okno_ram.delete('all')
                    czy3['wygenerowana_mapa3'] = False

        czy3 = {'wygenerowana_mapa3': False}
        okno_menu_algorytmy = tk.Toplevel(okno)
        okno_menu_algorytmy.title("Menu Algorytmów")
        okno_menu_algorytmy.minsize(height=str(810), width=str(1000))
        okno_menu_algorytmy.maxsize(height=str(810), width=str(1000))
        okno_menu_algorytmy.grab_set()
        okno_algorytmy = tk.Frame(okno_menu_algorytmy, height=str(810), width=str(1000))
       


        #TWORZENIE NAPISU I PRZYCISKÓW
        global roznica
        roznica = algorytm_1_czas - algorytm_2_czas
        tekst_algorytmy = tk.Label(okno_algorytmy, text="Menu Algorytmów", height=str(0), width=str(0), font = ("timesnewroman", 20, "bold"))
        przycisk_wybierz_mapę = tk.Button(okno_algorytmy, text="Wybierz mapę", width='15', height='3', font = ("timesnewroman", 15, 'bold'), command=lambda: przeglad_map(okno_algorytmy))
        przycisk_algorytm1 = tk.Button(okno_algorytmy, text="Algorytm D* Lite", width='15', height='3', font = ("timesnewroman", 15, 'bold'), command=lambda: dstarlite())
        przycisk_algorytm2 = tk.Button(okno_algorytmy, text="Algorytm LPA*", width='15', height='3', font = ("timesnewroman", 15, 'bold'), command=lambda: astar())
        przycisk_wyczyść = tk.Button(okno_algorytmy, text="Wyczyść", width='15', height='3', font = ("timesnewroman", 15, 'bold'), command=lambda: wyczysc())
        przycisk_powrót = tk.Button(okno_algorytmy, text="Powrót", width='15', height='3', font = ("timesnewroman", 15, 'bold'), command=lambda: okno_menu_algorytmy.destroy())

        #TWORZENIE KOMUNIKATU
        tekst_algorytmy.pack()
        tekst_algorytmy.place(x=SZEROKOSC_OKNA/2, y=WYSOKOSC_OKNA/2 - 250, anchor="center")
        przycisk_wybierz_mapę.pack()
        przycisk_wybierz_mapę.place(x=SZEROKOSC_OKNA/2 + 580, y=WYSOKOSC_OKNA/2 - 100, anchor="center")
        przycisk_algorytm1.pack()
        przycisk_algorytm1.place(x=SZEROKOSC_OKNA/2 + 580, y=WYSOKOSC_OKNA/2, anchor="center")
        przycisk_algorytm2.pack()
        przycisk_algorytm2.place(x=SZEROKOSC_OKNA/2 + 580, y=WYSOKOSC_OKNA/2 + 100, anchor="center")
        przycisk_wyczyść.pack()
        przycisk_wyczyść.place(x=SZEROKOSC_OKNA/2 + 580, y=WYSOKOSC_OKNA/2 + 200, anchor="center")
        przycisk_powrót.pack()
        przycisk_powrót.place(x=SZEROKOSC_OKNA/2 + 580, y=WYSOKOSC_OKNA/2 + 300, anchor="center")
        
        okno_algorytmy.pack()

    def okno_menu_mapy():
        '''
        Funkcja służąca do wywołania programu Program_optymalizaycjny
        Otworzenie Menu Mapy

        Funkcja nie przyjmuje żadnych zmiennych

        Funkcja nie zwraca żadnej wartości!!!!
        '''
        okno_mapy = tk.Toplevel(okno)
        okno_mapy.title("Menu Mapy")
        okno_mapy.minsize(height=str(810), width=str(1000))
        okno_mapy.maxsize(height=str(810), width=str(1000))
        okno_mapy.grab_set()
        plik_mapy.menu_mapy(okno_mapy)
        
    okno_menu_glowne = tk.Frame(okno, height=str(WYSOKOSC_OKNA), width=str(SZEROKOSC_OKNA))
  
    #TWORZENIE NAPISU I PRZYCISKÓW
    tekst_menu_glowne = tk.Label(okno_menu_glowne, text="MENU GŁÓWNE", height=str(0), width=str(0), font = ("timesnewroman", 20, "bold"))
    przycisk_menu_mapy = tk.Button(okno_menu_glowne, text="Menu Mapy", width='30', height='3', font = ("timesnewroman", 15, 'bold'), command=lambda: okno_menu_mapy())
    przycisk_Algorytmy = tk.Button(okno_menu_glowne, text="Algorytmy", width='30', height='3', font = ("timesnewroman", 15, 'bold'), command=lambda: okno_menu_algorytmy())
    przycisk_zakoncz = tk.Button(okno_menu_glowne, text="Zakończ", width='30', height='3', font = ("timesnewroman", 15, 'bold'), command=lambda: okno.destroy())

    tekst_menu_glowne.pack()
    tekst_menu_glowne.place(x=SZEROKOSC_OKNA/2, y=WYSOKOSC_OKNA/2 - 200, anchor="center")
    przycisk_menu_mapy.pack()
    przycisk_menu_mapy.place(x=SZEROKOSC_OKNA/2, y=WYSOKOSC_OKNA/2 - 100, anchor="center")
    przycisk_Algorytmy.pack()
    przycisk_Algorytmy.place(x=SZEROKOSC_OKNA/2, y=WYSOKOSC_OKNA/2, anchor="center")
    przycisk_zakoncz.pack()
    przycisk_zakoncz.place(x=SZEROKOSC_OKNA/2, y=WYSOKOSC_OKNA/2 + 100, anchor="center")

    okno_menu_glowne.pack()

menu_glowne()
okno.mainloop()