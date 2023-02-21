from ast import Break
import csv
import fnmatch
import os
from ast import Break
from click import command
import time
import os
import math
import random
import csv
import copy
from matplotlib.pyplot import close
import pandas as pd
from sys import displayhook
import pygame, sys
from pandas import wide_to_long
from gui import Animation
import tkinter as tk
import PIL
from pygame.locals import*
from PIL import Image, ImageDraw
from typing import Dict, List
from pyparsing import And
from d_star_lite import DStarLite
from grid import SLAM, OccupancyGridMap
from LPA import LPA
import matplotlib.pyplot as plt

OBSTACLEE = 128     #niewidoczna przeszkoda (szary kolor)
OBSTACLEEIR1 = 153  #widoczna przeszkoda (brązowy kolor)
OBSTACLEEIR2 = 76   #widoczna przeszkoda (brązowy kolor)
OBSTACLEEIR3 = 0    #widoczna przeszkoda (brązowy kolor)
UNOCCUPIED = 0      #widoczna przeszkoda (brązowy kolor)

#ZMIENNE GLOBALNE
WYSOKOSC_OKNA = 810
SZEROKOSC_OKNA = 1000
#TWORZENIE OKNA
global okno
pathobecny = os.path.realpath(os.path.join(os.getcwd(), os.path.dirname(__file__))) + "\\" + "mapy"

def liczba_czy_znak(zmienna):
    '''
    Funkcja służąca do sprawdzenia czy dana zmienna jest zapisana
    w postaci liczby czy ciągów znaków

    Funkcja przyjmuje zmienną typu string ("zmienna"), która jest sprawdzana

    Funkcja zwraca zmienną typu bool
    True jeśli zmienna jest liczbą
    False jeśli zmienna nie jest liczbą
    '''
    for i in range(0, len(zmienna)):
        if zmienna[i]=='0' or zmienna[i]=='1' or zmienna[i]=='2' or zmienna[i]=='3' or zmienna[i]=='4' or zmienna[i]=='5' or zmienna[i]=='6' or zmienna[i]=='7' or zmienna[i]=='8' or zmienna[i]=='9':
            tmpl = True
        else:
            tmpl = False
            break
    return tmpl

def animacja_dstar(sciezka_png, wybor):

            path_complete=[]
            rhspred=[]
            alg_time = 0
            
            view_range = 11
            gui = Animation(title="Path Planning",
                            width=6,
                            height=6,
                            margin=0,
                            x_dim=int(szerokosc_mapy),
                            y_dim=int(wysokosc_mapy),
                            start=start,
                            goal=goal,
                            viewing_range=view_range)
            pygame.event.clear()               
            visiblemap = OccupancyGridMap(x_dim=int(szerokosc_mapy),
                                           y_dim=int(wysokosc_mapy),
                                           exploration_setting='8N')
            new_position = start
            last_position = start
            mapa_szerokosc = str(szerokosc_mapy)
            mapa_wyskosc = str(wysokosc_mapy)
            mapa_pixele = []
            odczyt_mapa = PIL.Image.open(sciezka_png)
            odczyt_mapa_rgb = odczyt_mapa.convert("RGB")
            for i in range(0, (int(mapa_wyskosc))):
                mapa_pixele.append([])
                for j in range(0, (int(mapa_szerokosc))):
                    kolor_pixela = odczyt_mapa_rgb.getpixel((j, i))
                    mapa_pixele[i].append([0,0,0])
                    mapa_pixele[i][j][0] = kolor_pixela[0]
                    mapa_pixele[i][j][1] = kolor_pixela[1]
                    mapa_pixele[i][j][2] = kolor_pixela[2]    
                    if kolor_pixela[0] == OBSTACLEE and kolor_pixela[1] == OBSTACLEE and kolor_pixela[2] == OBSTACLEE:
                        (x, y) = (int(i), int(j))  
                        grid_cella = (x, y)
                        gui.world.set_obstacle(grid_cella)
                    elif kolor_pixela[0] == OBSTACLEEIR1 and kolor_pixela[1] == OBSTACLEEIR2 and kolor_pixela[2] == OBSTACLEEIR3:
                        (x, y) = (int(i), int(j))  
                        grid_cella = (x, y)
                        gui.world.set_obstaclei(grid_cella)  
                        visiblemap.set_obstaclei(grid_cella)

            if(wybor):         # D* Lite (optimized)
                dstar = DStarLite(map=visiblemap,
                                s_start=start,
                                s_goal=goal)
            else:              # LP A*
                dstar = LPA(map=visiblemap,s_start=start,s_goal=goal)
            slam = SLAM(map=visiblemap,
                        view_range=view_range)
            slam.set_ground_truth_map(gui.get_map())
            # symulowany ruch i obliczanie ścieżki
            path, g, rhs, time_elapsed = dstar.move_and_replan(robot_position=new_position)
            rhspred.append(rhs)
            alg_time+=time_elapsed
            SP = pygame.USEREVENT + 0
            pygame.time.set_timer(SP, 5)
            if(wybor):
                while not gui.done:
                    # aktualizacja mapy
                    # wyświetlanie ścieżki
                    if(gui.current==goal):
                        gui.run_game(path_complete)
                    else:
                        gui.run_game(path=path)
                                              
                    new_position = gui.current

                    if new_position != last_position:
                        path_complete.append(last_position)
                        last_position = new_position
                        new_edges_and_old_costs, slam_map = slam.rescan(global_position=new_position)
                        dstar.new_edges_and_old_costs = new_edges_and_old_costs
                        dstar.sensed_map = slam_map
                        path, g, rhs, time_elapsed= dstar.move_and_replan(robot_position=new_position)
                        alg_time+=time_elapsed
                            
                        if new_position==goal:
                            pygame.time.set_timer(SP, 0)
                            gui.active=False
                
            else:
               while not gui.done:
                               
                    if(gui.current==goal):
                        gui.run_game(path_complete)
                    else:
                        gui.run_game(path=path)
                        if len(path)>2:
                            path[0]=path[1]
                            del path[1]
                    new_position = gui.current
                    
                    if new_position != last_position:
                        path_complete.append(last_position)
                        last_position = new_position
                        new_edges_and_old_costs, slam_map = slam.rescan(global_position=new_position)
                        dstar.new_edges_and_old_costs = new_edges_and_old_costs
                        dstar.sensed_map = slam_map

                        if path[0]!=goal:
                            temptruth=True
                            pathrange=len(path)
                            for x in range(pathrange):
                                temptruth=temptruth*dstar.sensed_map.is_unoccupied(path[x])
                            if not temptruth:
                            #if not dstar.sensed_map.is_unoccupied(path[1]):
                                dstar.resetLPA(position=new_position)
                                path, g, rhs, time_elapsed = dstar.move_and_replan(robot_position=new_position)
                                rhspred.append(rhs)
                                alg_time+=time_elapsed

                        if new_position==goal or path==None:
                            pygame.time.set_timer(SP, 0)
                            gui.active=False

            # if(wybor):
            #     plt.imshow(rhs, cmap='hot', interpolation='nearest')
            # else:
            #     if(len(rhspred)>1):
            #         f, axarr = plt.subplots(len(rhspred),1)
            #         for i in range (0,len(rhspred)):
            #             axarr[i].imshow(rhspred[i], cmap='hot', interpolation='nearest')
            #     else:
            #         plt.imshow(rhspred[0], cmap='hot', interpolation='nearest')
            pygame.quit()
            # plt.show()
            print(path_complete)
            print(alg_time)
            return alg_time
  
def odczyt_mapy_z_png(nazwa_pliku_ppm, nazwa_pliku_png):
    '''
    Funkcja służąca do odczytu kolorów pixelów z pliku .png

    Funkcja przyjmuje zmienną typu string (nazwa_pliku_ppm),
    która jest ściażką do miejsca zapisu oraz nazwą mapy z rozszerzeniem .ppm
    Funkcja przyjmuje zmienną typu string (nazwa_pliku_png),
    która jest ściażką do miejsca zapisu oraz nazwą mapy z rozszerzeniem .png

    Funkcja zwraca zmienną typu słownik o wartościach 
    umożliwiających zapisanie mapy do pliku .ppm
    '''
    mapa_nazwa = nazwa_pliku_ppm
    mapa_standard = 'P3'
    mapa_szerokosc = str(szerokosc_mapy)
    mapa_wyskosc = str(wysokosc_mapy)
    mapa_szarosc = '255'
    mapa_pixele = []

    odczyt_mapa = PIL.Image.open(nazwa_pliku_png)
    odczyt_mapa_rgb = odczyt_mapa.convert("RGB")
    for i in range(0, (int(mapa_wyskosc))):
        mapa_pixele.append([])
        for j in range(0, (int(mapa_szerokosc))):
            kolor_pixela = odczyt_mapa_rgb.getpixel((j, i))
            mapa_pixele[i].append([0,0,0])
            mapa_pixele[i][j][0] = kolor_pixela[0]
            mapa_pixele[i][j][1] = kolor_pixela[1]
            mapa_pixele[i][j][2] = kolor_pixela[2]

    mapa = {'nazwa': mapa_nazwa, 'standard': mapa_standard, 'szerokosc': mapa_szerokosc,
             'wysokosc': mapa_wyskosc, 'szarosc': mapa_szarosc, 'pixele': mapa_pixele}
    odczyt_mapa.close()
    return mapa

def odczytaj_mape_z_csv(nazwa_pliku):
                
    '''
    Funkcja służąca odczytaniu informacji o elementach mapy do pliku .csv

    Funkcja przyjmuje zmienną typu string (nazwa_pliku),
    która jest ściażką do miejsca odczytu oraz nazwą mapy z rozszerzeniem .csv

    Funkcja nie zwraca żadnej wartości
    '''
    
    with open(nazwa_pliku, 'r') as f:
        reader = csv.DictReader(f, delimiter=';')
        for row in reader:
            print(row['Tag'])
            if row['Tag']=='punkt_docelowy':
                global goal
                wspxy = [float(row['WspolrzednaX']), float(row['WspolrzednaY'])]
                goal = (int(wspxy[1]),int(wspxy[0]))
                print(wspxy[0])
                print(wspxy[1])
            if row['Tag']=='robot':   
                global start
                wspx = [float(row['WspolrzednaX']), float(row['WspolrzednaY'])]
                start = (int(wspx[1]),int(wspx[0]))
                print(wspx[0])
                print(wspx[1])
    close(nazwa_pliku)


def zapis_mapy_do_ppm(mapa, nazwa_pliku):
    '''
    Funkcja służąca do stworzenia pliku .ppm o wartościach z słownika "mapa"

    Funkcja przyjmuje zmienną typu słownik ("mapa") 
    o wartościach umożliwiających zapis do pliku .ppm
    Funkcja przyjmuje zmienną typu string (nazwa_pliku),
    która jest ściażką do miejsca zapisu oraz nazwą mapy z rozszerzeniem .png
    
    Funkcja niezwraca żadnej zmiennej
    '''
    mapa_nazwa = nazwa_pliku
    zapis = open(mapa_nazwa, mode='w')
    zapis.write(f"{mapa['standard']}\n")
    zapis.write(f"{mapa['szerokosc']} {mapa['wysokosc']}\n")
    zapis.write(f"{mapa['szarosc']}\n")
    for i in range(0, (int(mapa["wysokosc"]))):
        for j in range(0, (int(mapa["szerokosc"]))):
            zapis.write(f"{mapa['pixele'][i][j][0]} ")
            zapis.write(f"{mapa['pixele'][i][j][1]} ")
            zapis.write(f"{mapa['pixele'][i][j][2]} ")
        zapis.write(f"\n")
    zapis.close()
    '''
    Funkcja służąca do stworzenia pliku .ppm o wartościach z słownika "mapa"

    Funkcja przyjmuje zmienną typu słownik ("mapa") 
    o wartościach umożliwiających zapis do pliku .ppm
    
    Funkcja niezwraca żadnej zmiennej
    '''

    '''
    mapa_nazwa = input("Podaj nazwę mapy: ")
    mapa_nazwa = mapa_nazwa + ".ppm"
    '''
    mapa_nazwa = 'nowy.ppm'
    zapis = open(mapa_nazwa, mode='w')
    zapis.write(f"{mapa['standard']}\n")
    zapis.write(f"{mapa['szerokosc']} {mapa['wysokosc']}\n")
    zapis.write(f"{mapa['szarosc']}\n")
    for i in range(0, (int(mapa["wysokosc"]))):
        for j in range(0, (int(mapa["szerokosc"]))):
            zapis.write(f"{mapa['pixele'][i][j][0]} ")
            zapis.write(f"{mapa['pixele'][i][j][1]} ")
            zapis.write(f"{mapa['pixele'][i][j][2]} ")
        zapis.write(f"\n")
    zapis.close()

def odczyt_mapy_z_ppm(mapa_nazwa):
    '''
    Funkcja służąca do odczytu mapy z pliku .ppm

    Funkcja przyjmuje zmienną typu string (mapa_nazwa),
    która jest ścieżką dostępu do odczytywanego pliku

    Funkcja zwraca zmienną typu słownik o wartościach 
    umożliwiających utworzenie mapy
    '''
    '''
    mapa_nazwa = input("Podaj nazwę mapy: ")
    mapa_nazwa = mapa_nazwa + ".PPM"
    ''' 
    odczyt = open(mapa_nazwa, mode='r')
    plik = odczyt.readlines()
    odczyt.close()
    mapa_pixele = []
    tmpl_pixele = []
    for wiersz in range(1, len(plik)+1):
        tmpl = plik[wiersz-1].strip()
        if wiersz == 1:
            mapa_standard = tmpl.strip()
        elif wiersz == 2:
            tmpl = tmpl.strip()
            mapa_rozdzielczosc = tmpl.split(' ')
            mapa_szerokosc = mapa_rozdzielczosc[0]
            global szerokosc_mapy
            szerokosc_mapy = mapa_rozdzielczosc[0]
            mapa_wyskosc = mapa_rozdzielczosc[1]
            global wysokosc_mapy
            wysokosc_mapy = mapa_rozdzielczosc[0]
        elif wiersz == 3:
            mapa_szarosc = tmpl.strip()
        else:
            tmpl = tmpl.strip()
            tmpl = tmpl.split(' ')
            tmpl_pixele.append(tmpl)
    for i in range(0, (int(mapa_wyskosc))):
        mapa_pixele.append([])
        tmpl_2 = 0
        for j in range(0, (int(mapa_szerokosc))):
            mapa_pixele[i].append([])
            mapa_pixele[i][j].append(tmpl_pixele[i][tmpl_2])
            mapa_pixele[i][j].append(tmpl_pixele[i][tmpl_2+1])
            mapa_pixele[i][j].append(tmpl_pixele[i][tmpl_2+2])
            tmpl_2 += 3
    mapa = {'nazwa': mapa_nazwa, 'standard': mapa_standard, 'szerokosc': mapa_szerokosc,
             'wysokosc': mapa_wyskosc, 'szarosc': mapa_szarosc, 'pixele': mapa_pixele}
    return mapa

def menu_mapy(okno):

    def stworz_mape(okno_stare):
            '''
            Funkcja służąca tworzenia mapy dla poruszającego się robota.
            Funkcja zapisuje mapę w postaci pliku .png oraz .ppm

            Funkcja przyjmuje zmienną typu Frame ("okno_stare"),
            które jest oknem, które ma zostać dezaktywowane

            Funkcja nie zwraca żadnej wartości
            '''
            def sprawdzenie_poprawnosci():
                '''
                Funkcja służąca sprawdzenia czy podane parametry mapy 
                są właściwie zapisane i podane w odpowiednim przedziale

                Funkcja nie zwraca żadnej wartości
                '''
                poprawnosc = False
                global wysokosc_mapy
                global szerokosc_mapy
                wysokosc_mapy = str(wpisz_mapa_wysokosc.get())
                szerokosc_mapy = str(wpisz_mapa_szerokosc.get())
                if len(wysokosc_mapy) > 0 and len(szerokosc_mapy) > 0:
                    wysokosc_liczba = liczba_czy_znak(wysokosc_mapy)
                    szerokosc_liczba = liczba_czy_znak(szerokosc_mapy)
                    if wysokosc_liczba == True and szerokosc_liczba == True:
                        wysokosc_mapy = int(wysokosc_mapy)
                        szerokosc_mapy = int(szerokosc_mapy)
                        if 801 > wysokosc_mapy >  99 and 801 > szerokosc_mapy >  99:
                            poprawnosc = True
                        else:
                            tekst_informacji.set("Wprowadzono za małą bądź za dużą liczbę w co najmniej jednym z parametrów mapy")
                    else:
                        tekst_informacji.set("Wprowadzono źle co najmniej jedną z parametrów mapy!!!")
                else:
                    tekst_informacji.set("Nie wprowadzono co najmniej jednej z parametrów mapy!!!")
                return poprawnosc

            def generuj_mape():
                '''
                Funkcja służąca generowaniu mapy o odpowiednim rozmiarze w oknie programu
                oraz służąca do zmiany wielkości mapy i ponowym jej wygenerowaniu na ekranie w oknie programu

                Funkcja nie zwraca żadnej wartości
                '''
                if czy['wygenerowana_mapa'] == False:
                    if (sprawdzenie_poprawnosci() == True):
                        global okno_mapa
                        global okno_ramka
                        wpisz_mapa_wysokosc.delete(0, 'end')
                        wpisz_mapa_szerokosc.delete(0, 'end')
                        okno_ramka = tk.Canvas(okno_tworzenie_mapy, height=str(wysokosc_mapy+10), width=str(szerokosc_mapy+10), highlightthickness=0)
                        okno_ramka.create_rectangle(0, 0, szerokosc_mapy+10 ,wysokosc_mapy+10, fill="black", width=0)
                        okno_mapa = tk.Canvas(okno_tworzenie_mapy, height=str(wysokosc_mapy), width=str(szerokosc_mapy), highlightthickness=0, bg="black")
                        okno_mapa.create_rectangle(0, 0, szerokosc_mapy , wysokosc_mapy, fill="white", width=0, tag='mapa')
                        okno_ramka.pack()
                        okno_ramka.place(x=405, y=405, anchor="center")
                        okno_mapa.pack()
                        okno_mapa.place(x=405, y=405, anchor="center")
                        czy['wygenerowana_mapa'] = True
                        tekst_informacji.set("Umieść robota na mapie:\n1.Naciśnij przycisk 'Stwórz robota'\n"
                                            "2.Zaznacz miejsce umieszczenia robota na mapie")
                else:
                    if (sprawdzenie_poprawnosci() == True):
                        okno_mapa.delete('all')
                        okno_ramka.delete('all')
                        okno_mapa.destroy()
                        okno_ramka.destroy()
                        czy['wygenerowana_mapa'] = False
                        czy['wygenerowany_robot'] = False
                        czy['wygenerowany_punkt'] = False
                        czy['wygenerowana_przeszkoda_widoczna'].clear()
                        czy['wygenerowana_przeszkoda_widoczna'] = []
                        czy['wygenerowana_przeszkoda_widoczna'].append(False)
                        czy['wygenerowana_przeszkoda_niewidoczna'].clear()
                        czy['wygenerowana_przeszkoda_niewidoczna'] = []
                        czy['wygenerowana_przeszkoda_niewidoczna'].append(False)
                        czy['usunieta_przeszkoda'].clear()
                        czy['usunieta_przeszkoda'] = []
                        czy['usunieta_przeszkoda'].append(False)
                        generuj_mape()

            def tworzenie_wspolrzednych(x, y, szerokosc_mapy, wysokosc_mapy):
                '''
                Funkcja służąca generowaniu wspołrzędnych robota i punktu docelowego
                gdy wskażemy wspołrzędne, ktore spowodowałyby utworzenie niecałego robota/pola docelowego

                Funkcja przyjmuje zmienną typu int ("x"), która jest wartością wspołrzędnej X miejsca kliknięcia myszy
                Funkcja przyjmuje zmienną typu int ("y"), która jest wartością wspołrzędnej Y miejsca kliknięcia myszy
                Funkcja przyjmuje zmienną typu int ("szerokosc_mapy"), wartość szerokości mapy
                Funkcja przyjmuje zmienną typu int ("wysokosc_mapy"), wartość wysokości mapy

                Funkcja zwraca zmienne typu int (x_pocz, y_pocz, x_kon, y_kon), które są
                wartością początkową oraz wartością końcową wspołrzędnych X i Y
                koniecznych do stworzenia robota/pola docelowego
                '''
                if x < 10:
                    x_pocz = 0
                    x_kon = 20
                elif x > szerokosc_mapy - 11:
                    x_pocz = szerokosc_mapy - 21
                    x_kon = szerokosc_mapy - 1
                else:
                    x_pocz = x - 10
                    x_kon = x + 10
                if y < 10:
                    y_pocz = 0
                    y_kon = 20
                elif y > wysokosc_mapy - 11:
                    y_pocz = wysokosc_mapy - 21
                    y_kon = wysokosc_mapy - 1
                else:
                    y_pocz = y - 10
                    y_kon = y + 10
                return x_pocz, y_pocz, x_kon, y_kon

            def tworzenie_robota(event):
                '''
                Funkcja służąca do tworzenia robota

                Funkcja przyjmuje zmienną ("event"), która oznacza kliknięcie myszy
                '''
                
                x_pocz, y_pocz, x_kon, y_kon = tworzenie_wspolrzednych(event.x, event.y, szerokosc_mapy, wysokosc_mapy)
                okno_mapa.create_oval(x_pocz, y_pocz, x_kon, y_kon, fill="green", width=0, tag='robot')
                czy['wygenerowany_robot'] = True
                okno_mapa.unbind("<Button-1>")

            def generuj_robota():
                '''
                Funkcja służąca do generowania robota, w miejscu w którym klikniemy myszą

                Funkcja nie zwraca żadnej wartości
                '''
                if czy['wygenerowana_mapa'] == True:
                    if czy['wygenerowany_robot'] == False:
                        okno_mapa.bind("<Button-1>", tworzenie_robota)
                        tekst_informacji.set("Umieść punkt docelowy na mapie:\n1.Naciśnij przycisk 'Stwórz punkt docelowy'\n"
                                                "2.Zaznacz miejsce umieszczenia punktu docelowego na mapie")
                    else:
                        tekst_informacji.set("Wygenerowano już robota!!!")
                else:
                    tekst_informacji.set("By wygenerować robota wygeneruj pierw mapę!!!\n\n"
                                            "1.Podaj wartość  szerokość i wysokość z przedziału od 100 do 800\n"
                                            "2.Naciśnij przycisk 'Stwórz mapę'")

            def usun_robota():
                '''
                Funkcja służąca usunięciu robota z mapy

                Funkcja nie zwraca żadnej wartości
                '''
                if czy['wygenerowany_robot'] == True:
                    okno_mapa.delete('robot')
                    czy['wygenerowany_robot'] = False
                else:
                    tekst_informacji.set("Nie stworzono jeszcze robota!!!")

            def tworzenie_punktu(event):
                '''
                Funkcja służąca do tworzenia punktu docelowego

                Funkcja przyjmuje zmienną ("event"), która oznacza kliknięcie myszy
                '''
                x_pocz, y_pocz, x_kon, y_kon = tworzenie_wspolrzednych(event.x, event.y, szerokosc_mapy, wysokosc_mapy)
                okno_mapa.create_oval(x_pocz, y_pocz, x_kon, y_kon, fill="red", width=0, tag='punkt_docelowy')
                czy['wygenerowany_punkt'] = True
                okno_mapa.unbind("<Button-1>")

            def generuj_punkt():
                '''
                Funkcja służąca do generowania punktu docelowego, 
                w miejscu w którym klikniemy myszą

                Funkcja nie zwraca żadnej wartości
                '''
                if czy['wygenerowana_mapa'] == True:
                    if czy['wygenerowany_punkt'] == False:
                        okno_mapa.bind("<Button-1>", tworzenie_punktu)
                        tekst_informacji.set("Umieść przeszkody na mapie:\n1.Naciśnij przycisk 'Stwórz przeszkodę'\n"
                                                "2.Zaznacz kontur przeszkody na mapie\nNaciśnij 'Enter' na klawiaturze")
                    else:
                        tekst_informacji.set("Wygenerowano już punkt docelowy!!!")
                else:
                    tekst_informacji.set("By wygenerować punkt docelowy wygeneruj pierw mapę!!!\n\n"
                                            "1.Podaj wartość  szerokość i wysokość z przedziału od 100 do 800\n"
                                            "2.Naciśnij przycisk 'Stwórz mapę'")

            def usun_punkt():
                '''
                Funkcja służąca usunięciu punktu docelowego z mapy

                Funkcja nie zwraca żadnej wartości
                '''
                if czy['wygenerowany_punkt'] == True:
                    okno_mapa.delete('punkt_docelowy')
                    czy['wygenerowany_punkt'] = False
                else:
                    tekst_informacji.set("Nie stworzono jeszcze punktu docelowego!!!")

            def dodaj_wspolrzedne(event, kolor):
                '''
                Funkcja służąca dodaniu kolejnych wspołrzędnych, 
                w celu uzyskania przeszkody w kształcie wielokąta

                Funkcja przyjmuje zmienną ("event"), która oznacza kliknięcie myszy

                Funkcja nie zwraca żadnej wartości
                '''
                wspolrzedne.append(event.x)
                wspolrzedne.append(event.y)
                if len(wspolrzedne) > 2:
                    okno_mapa.create_line(wspolrzedne[len(wspolrzedne)-2], wspolrzedne[len(wspolrzedne)-1],
                                            wspolrzedne[len(wspolrzedne)-4], wspolrzedne[len(wspolrzedne)-3], fill=kolor, tag='linie')

            def rysuj_linie(event, kolor):
                '''
                Funkcja służąca stworzeniu na mapie tymczasowych liń,
                w celu łatwiejszego stworzenia przeszkody w postaci wielokąta, 

                Funkcja przyjmuje zmienną ("event"), która oznacza kliknięcie myszy
                Funkcja przyjmuje zmienną string ("kolor"), która oznacza kolor tworzenia liń

                Funkcja nie zwraca żadnej wartości
                '''
                okno_mapa.delete('linia_1')
                okno_mapa.delete('linia_2')
                wspolrzedne_do_rysowania_x = event.x
                wspolrzedne_do_rysowania_y = event.y
                if len(wspolrzedne) > 1:
                    okno_mapa.create_line(wspolrzedne[len(wspolrzedne)-2], wspolrzedne[len(wspolrzedne)-1],
                                            wspolrzedne_do_rysowania_x, wspolrzedne_do_rysowania_y, fill=kolor, tag="linia_1")
                if len(wspolrzedne) > 1:
                    okno_mapa.create_line(wspolrzedne[0], wspolrzedne[1], wspolrzedne_do_rysowania_x, wspolrzedne_do_rysowania_y, fill=kolor, tag="linia_2")

            def tworzenie_kształtu(event, kolor, tryb):
                '''
                Funkcja służąca stworzeniu na mapie przeszkody w postaci wielokąta 

                Funkcja przyjmuje zmienną ("event"), która oznacza kliknięcie myszy
                Funkcja przyjmuje zmienną string ("kolor"), która oznacza kolor przeszkody
                Funkcja przyjmuje zmienną string ("tryb"), która określa 
                czy przeszkoda zostaje stworzona czy usunięta

                Funkcja nie zwraca żadnej wartości
                '''
                if tryb == 'tworzenie':
                    if kolor == '#994C00':
                        tmpl_text = "przeszkoda_widoczna" + str(len(czy['wygenerowana_przeszkoda_widoczna']))
                        czy['wygenerowana_przeszkoda_widoczna'][len(czy['wygenerowana_przeszkoda_widoczna'])-1] = True
                        czy['wygenerowana_przeszkoda_widoczna'].append(False)
                    if kolor == 'gray':
                        tmpl_text = "przeszkoda_niewidoczna" + str(len(czy['wygenerowana_przeszkoda_niewidoczna']))
                        czy['wygenerowana_przeszkoda_niewidoczna'][len(czy['wygenerowana_przeszkoda_niewidoczna'])-1] = True
                        czy['wygenerowana_przeszkoda_niewidoczna'].append(False)
                else:
                    tmpl_text = "usunieta_przeszkoda_" + str(len(czy['usunieta_przeszkoda']))
                    czy['usunieta_przeszkoda'][len(czy['usunieta_przeszkoda'])-1] = True
                    czy['usunieta_przeszkoda'].append(False)
                okno_mapa.create_polygon(wspolrzedne, fill=kolor, tag=tmpl_text)
                okno_mapa.unbind("<Button-1>")
                okno_mapa.unbind("<Motion>")
                okno.unbind("<Return>")
                okno_mapa.delete('linia_1')
                okno_mapa.delete('linia_2')
                okno_mapa.delete('linie')
                wspolrzedne.clear()

            def generuj_przeszkode(rodzaj):
                '''
                Funkcja służąca generowaniu przeszkody na mapie
                przy użyciu myszy i klawiatury

                Funkcja przyjmuje zmienną string ("rodzaj"), 
                która oznacza czy przeszkoda jest widoczna czy nie

                Funkcja nie zwraca żadnej wartości
                '''
                if czy['wygenerowana_mapa'] == True:
                    okno_mapa.bind("<Button-1>", lambda event, kolor = rodzaj:
                                    dodaj_wspolrzedne(event, kolor))
                    okno_mapa.bind("<Motion>", lambda event, kolor = rodzaj:
                                    rysuj_linie(event, kolor))
                    okno.bind("<Return>", lambda event, kolor = rodzaj, tryb = 'tworzenie':
                                tworzenie_kształtu(event, kolor, tryb))
                    tekst_informacji.set("Jeśli chcesz zapisać mapę naciśnij przycisk 'Zapisz mapę'\n\n"
                                            "Jeśli chcesz edytować mapę naciśnij odpowiedni przycisk funkcji edycj")
                else:
                    tekst_informacji.set("By wygenerować przeszkodę wygeneruj pierw mapę!!!\n\n"
                                            "1.Podaj wartość  szerokość i wysokość z przedziału od 100 do 800\n"
                                            "2.Naciśnij przycisk 'Stwórz mapę'")

            def usun_przeszkode():
                '''
                Funkcja służąca usuwaniu przeszkody na mapie
                przy użyciu myszy i klawiatury

                Funkcja nie zwraca żadnej wartości
                '''
                if czy['wygenerowana_mapa'] == True:
                    if czy['wygenerowana_przeszkoda_widoczna'][0] == True or czy['wygenerowana_przeszkoda_niewidoczna'][0] == True:
                        okno_mapa.bind("<Button-1>", lambda event, kolor = 'black':
                                        dodaj_wspolrzedne(event, kolor))
                        okno_mapa.bind("<Motion>", lambda event, kolor = 'black':
                                        rysuj_linie(event, kolor))
                        okno.bind("<Return>", lambda event, kolor = 'white', tryb = 'usuwanie':
                                    tworzenie_kształtu(event, kolor, tryb))
                        tekst_informacji.set("Jeśli chcesz zapisać mapę naciśnij przycisk 'Zapisz mapę'\n\n"
                                                "Jeśli chcesz edytować mapę naciśnij odpowiedni przycisk funkcji edycj")
                    else:
                        tekst_informacji.set("Nie stworzono jeszcze przeszkody!!!")
                else:
                    tekst_informacji.set("By usunąć przeszkodę wygeneruj pierw mapę!!!\n\n"
                                            "1.Podaj wartość  szerokość i wysokość z przedziału od 100 do 800\n"
                                            "2.Naciśnij przycisk 'Stwórz mapę'")

            def zapisz_mape_do_png(nazwa_pliku):
                '''
                Funkcja służąca zapisaniu stworzonej mapy z wszystkimi elementami do pliku .png

                Funkcja przyjmuje zmienną typu string (nazwa_pliku),
                która jest ściażką do miejsca zapisu oraz nazwą mapy z rozszerzeniem .png

                Funkcja nie zwraca żadnej wartości
                '''
                image = PIL.Image.new("RGB", (szerokosc_mapy, wysokosc_mapy))
                rysuj = ImageDraw.Draw(image)
                wspolrzedne_rysowania = []
                id_elementow_mapy = okno_mapa.find_all()
                for j in id_elementow_mapy:
                    typ_elementu = okno_mapa.type(j)
                    tag_elementu = okno_mapa.gettags(j)
                    wspolrzedne_canvas = okno_mapa.coords(j)
                    if typ_elementu == 'rectangle':
                        for i in range (0, len(wspolrzedne_canvas)):
                            if i < 2:
                                wspolrzedne_rysowania.append(wspolrzedne_canvas[i])
                            else:
                                if wspolrzedne_canvas[i] == 0:
                                    wspolrzedne_rysowania.append(wspolrzedne_canvas[i])
                                else:
                                    wspolrzedne_rysowania.append(wspolrzedne_canvas[i] - 1)
                        rysuj.rectangle(wspolrzedne_canvas, fill = "white")                         
                    elif typ_elementu == 'oval':
                        if tag_elementu[0] == 'robot':
                            for i in range (0, len(wspolrzedne_canvas)):
                                wspolrzedne_rysowania.append(wspolrzedne_canvas[i])
                            rysuj.ellipse(wspolrzedne_canvas, fill = "green")
                        elif tag_elementu[0] == 'punkt_docelowy':
                            for i in range (0, len(wspolrzedne_canvas)):
                                wspolrzedne_rysowania.append(wspolrzedne_canvas[i])
                            rysuj.ellipse(wspolrzedne_canvas, fill = "red")
                    elif typ_elementu == 'polygon':
                        if tag_elementu[0][0] == 'p':
                            if tag_elementu[0][11] == 'w':
                                for i in range (0, len(wspolrzedne_canvas)):
                                    if i < 2:
                                        wspolrzedne_rysowania.append(wspolrzedne_canvas[i])
                                    else:
                                        if wspolrzedne_canvas[i] == 0:
                                            wspolrzedne_rysowania.append(wspolrzedne_canvas[i])
                                        else:
                                            wspolrzedne_rysowania.append(wspolrzedne_canvas[i] - 1)
                                rysuj.polygon(wspolrzedne_canvas, fill = "#994C00")
                            elif tag_elementu[0][11] == 'n':
                                for i in range (0, len(wspolrzedne_canvas)):
                                    if i < 2:
                                        wspolrzedne_rysowania.append(wspolrzedne_canvas[i])
                                    else:
                                        if wspolrzedne_canvas[i] == 0:
                                            wspolrzedne_rysowania.append(wspolrzedne_canvas[i])
                                        else:
                                            wspolrzedne_rysowania.append(wspolrzedne_canvas[i] - 1)
                                rysuj.polygon(wspolrzedne_canvas, fill = "gray")
                        elif tag_elementu[0][0] == 'u':
                            for i in range (0, len(wspolrzedne_canvas)):
                                if i < 2:
                                    wspolrzedne_rysowania.append(wspolrzedne_canvas[i])
                                else:
                                    if wspolrzedne_canvas[i] == 0:
                                        wspolrzedne_rysowania.append(wspolrzedne_canvas[i])
                                    else:
                                        wspolrzedne_rysowania.append(wspolrzedne_canvas[i] - 1)
                            rysuj.polygon(wspolrzedne_canvas, fill = "white")
                    wspolrzedne_rysowania.clear()
                    wspolrzedne_canvas.clear()  
                image.save(nazwa_pliku)
            
           
            def zapisz_mape_do_csv(nazwa_pliku):

                '''
                Funkcja służąca zapisaniu informacji o elementach mapy do pliku .csv

                Funkcja przyjmuje zmienną typu string (nazwa_pliku),
                która jest ściażką do miejsca zapisu oraz nazwą mapy z rozszerzeniem .csv

                Funkcja nie zwraca żadnej wartości
                '''
                zapis = open(nazwa_pliku, 'w')
                id_elementow_mapy = okno_mapa.find_all()
                zapis.write(f"{'Id'};{'Tag'};{'Typ'};{'Kolor'};{'Wspolrzedne'};{'WspolrzednaX'};{'WspolrzednaY'}\n")
                for i in id_elementow_mapy:
                    Id = i
                    Tag = okno_mapa.gettags(i)
                    print(Id)
                    print(Tag)
                    Typ = okno_mapa.type(i)
                    Kolor = okno_mapa.itemcget(i, "fill")
                    Cords = okno_mapa.coords(i)
                    if Tag[0] == 'robot' or Tag[0] == 'punkt_docelowy':
                      print('YOLO')
                      print(Cords)
                      CordX = (Cords[2]-10)
                      CordY = (Cords[3]-10)
                      print(Cords[0])
                    else:
                      CordX = 0
                      CordY = 0
                    zapis.write(f'{Id};{Tag[0]};{Typ};{Kolor};{Cords};{CordX};{CordY}\n')
                zapis.close()

           

            
            def proces_zapisania(okno_zapisu, okno_aktualne, wpisz_nazwa_pliku):
                '''
                Funkcja służąca zapisaniu stworzonej mapy z wszystkimi elementami do pliku .png,
                odczytaniu kolorów z pliku .png i zapisania wszystkiego do pliku .ppm

                Funkcja przyjmuje zmienną typu TopLevel (okno_zapisu),
                która jest nowym oknem wyskakującym
                Funkcja przyjmuje zmienną typu Frame (okno_aktualne),
                która jest oknem do podania nazwy mapy
                Funkcja przyjmuje zmienną typu Entry (wpisz_nawa_pliku),
                która jest miejscem wpisania nazwy mapy

                Funkcja nie zwraca żadnej wartości
                '''
                nazwa_pliku = wpisz_nazwa_pliku.get()
                wpisz_nazwa_pliku.delete(0, 'end')
                nazwa_ppm = "C:\\Users\\User\\OneDrive - Politechnika Wroclawska\\ISA Semestr1\\AO-projekt\\projektpodlogamapy" + '\\' + nazwa_pliku + '.ppm'
                nazwa_png = "C:\\Users\\User\\OneDrive - Politechnika Wroclawska\\ISA Semestr1\\AO-projekt\\projektpodlogamapy" + '\\' + nazwa_pliku + '.png'
                nazwa_csv = "C:\\Users\\User\\OneDrive - Politechnika Wroclawska\\ISA Semestr1\\AO-projekt\\projektpodlogamapy" + '\\' + nazwa_pliku + '.csv'
                zapisz_mape_do_png(nazwa_png)
                mapa = odczyt_mapy_z_png(nazwa_ppm, nazwa_png)
                zapis_mapy_do_ppm(mapa, nazwa_ppm)
                zapisz_mape_do_csv(nazwa_csv)
                okno_aktualne.pack_forget()
                okno_zapisano_mape = tk.Frame(okno_zapisu)
                tekst_zapisano_mape = tk.Label(okno_zapisano_mape, text="Mapa została zapisana!!!", font = ("timesnewroman", 15, 'bold'))
                tekst_zapisano_mape.pack(pady=50, anchor='center')
                okno_zapisano_mape.pack()

            def zapisz():
                '''
                Funkcja służąca zapisaniu stworzonej mapy z wszystkimi elementami do pliku .png,
                odczytaniu kolorów z pliku .png i zapisania wszystkiego do pliku .ppm

                Funkcja nie zwraca żadnej wartości
                '''
                if czy['wygenerowana_mapa'] == True:
                    okno_zapisu = tk.Toplevel(okno)
                    okno_zapisu.title("Zapis mapy")
                    okno_zapisu.minsize(height='150', width='300')
                    okno_zapisu.maxsize(height='150', width='300')
                    okno_edycji_zapisu = tk.Frame(okno_zapisu)
                    tekst_nazwa_pliku = tk.Label(okno_edycji_zapisu, text="Podaj nazwę mapy:", font = ("timesnewroman", 12))
                    wpisz_nazwa_pliku = tk.Entry(okno_edycji_zapisu, width='20', font = ("timesnewroman", 10))
                    przycisk_zapisz_plik = tk.Button(okno_edycji_zapisu, text="Zapisz mapę", width='15', font = ("timesnewroman", 12), command=lambda: proces_zapisania(okno_zapisu, okno_edycji_zapisu, wpisz_nazwa_pliku))

                    tekst_nazwa_pliku.pack(pady=10)
                    wpisz_nazwa_pliku.pack(pady=2)
                    przycisk_zapisz_plik.pack(pady=10)

                    okno_edycji_zapisu.pack()
                    okno_zapisu.mainloop()
                else:
                    tekst_informacji.set("Nie wygenerowano jeszcze mapy!!!")

            def powrot_menu_mapy(okno_aktualne, okno_nowe):
                '''
                Funkcja służąca do powrotu do manu mapy

                Funkcja przyjmuje zmienną typu Frame ("okno_aktualne"),
                które jest oknem, które ma zostać dezaktywowane
                Funkcja przyjmuje zmienną typu Frame ("okno_nowe"),
                które jest oknem, które ma zostać aktywowane

                Funkcja nie zwraca żadnej wartości
                '''
                okno_aktualne.pack_forget()
                okno_nowe.pack()
                if czy['wygenerowana_mapa'] == True:
                    okno_mapa.delete('all')
                    okno_ramka.delete('all')
                    okno_mapa.destroy()
                    okno_ramka.destroy()


            #ZMIENNE
            okno_stare.pack_forget()
            okno_tworzenie_mapy = tk.Frame(okno, height=str(WYSOKOSC_OKNA), width=str(SZEROKOSC_OKNA))
            tekst_informacji = tk.StringVar()
            czy = {'wygenerowana_mapa': False, 'wygenerowany_robot': False, 'wygenerowany_punkt': False, 'wygenerowana_przeszkoda_widoczna': [False], 
                    'wygenerowana_przeszkoda_niewidoczna': [False], 'usunieta_przeszkoda': [False]}
            wspolrzedne = []

            #TWORZENIE KOMUNIKATU
            tekst_informacji.set("Podaj wielkość mapy\n1.Podaj wartość  szerokość i wysokość z przedziału od 100 do 800\n2.Naciśnij przycisk 'Stwórz mapę'")
            informacja_tytul = tk.Message(okno_tworzenie_mapy, width='180', font = ("timesnewroman", 12, 'bold'), text="Informacje:")
            informacja = tk.Message(okno_tworzenie_mapy, width='180', font = ("timesnewroman", 10), textvariable=tekst_informacji)
            #TWORZENIE MENU EDYCJI MAPY
            tekst_wielkosc_mapy = tk.Label(okno_tworzenie_mapy, text="Rozmiar mapy", height=str(0), width=str(0), font = ("timesnewroman", 12, "bold"))
            tekst_mapa_wysokosc = tk.Label(okno_tworzenie_mapy, text="Wysokosc", font = ("timesnewroman", 10))
            tekst_mapa_szerokosc = tk.Label(okno_tworzenie_mapy, text="Szerokosc", font = ("timesnewroman", 10))
            wpisz_mapa_wysokosc = tk.Entry(okno_tworzenie_mapy, width='15', font = ("timesnewroman", 10))
            wpisz_mapa_szerokosc = tk.Entry(okno_tworzenie_mapy, width='15', font = ("timesnewroman", 10))
            #TWORZENIE PRZYCISKÓW
            przycisk_stworz_mape = tk.Button(okno_tworzenie_mapy, text="Stwórz mapę", width='15', font = ("timesnewroman", 15), command=lambda: generuj_mape())
            przycisk_stworz_robota = tk.Button(okno_tworzenie_mapy, text="Stwórz robota", width='15', font = ("timesnewroman", 15), command=lambda: generuj_robota())
            przycisk_usun_robota = tk.Button(okno_tworzenie_mapy, text="Usuń robota", width='15', font = ("timesnewroman", 15), command=lambda: usun_robota())
            przycisk_stworz_punkt = tk.Button(okno_tworzenie_mapy, text="Stwórz\npunkt docelowy", width='15', font = ("timesnewroman", 15), command=lambda: generuj_punkt())
            przycisk_usun_punkt = tk.Button(okno_tworzenie_mapy, text="Usuń \npunkt docelowy", width='15', font = ("timesnewroman", 15), command=lambda: usun_punkt())
            przycisk_stworz_przeszkode_widoczną = tk.Button(okno_tworzenie_mapy, text="Stwórz przeszkodę\nwidoczną", width='15',font = ("timesnewroman", 15), command=lambda: generuj_przeszkode('#994C00'))
            przycisk_stworz_przeszkode_niewidoczną = tk.Button(okno_tworzenie_mapy, text="Stwórz przeszkodę\nniewidoczną", width='15',font = ("timesnewroman", 15), command=lambda: generuj_przeszkode('gray'))
            przycisk_usun_przeszkode = tk.Button(okno_tworzenie_mapy, text="Usuń przeszkodę", width='15', font = ("timesnewroman", 15), command=lambda: usun_przeszkode())
            przycisk_zapisz_mape = tk.Button(okno_tworzenie_mapy, text="Zapisz mapę", width='15', font = ("timesnewroman", 15), command=lambda: zapisz())
            przycisk_wyjscie = tk.Button(okno_tworzenie_mapy, text="Powrót do menu", width='15', font = ("timesnewroman", 15), command=lambda: powrot_menu_mapy(okno_tworzenie_mapy, okno_menu_mapy))

            informacja_tytul.pack()
            informacja_tytul.place(x=810, y=0, anchor="nw")
            informacja.pack()
            informacja.place(x=810, y=25, anchor="nw")
            #WIELKOŚĆ MAPY
            tekst_wielkosc_mapy.pack()
            tekst_wielkosc_mapy.place(x=905, y=167, anchor="center")
            tekst_mapa_wysokosc.pack()
            tekst_mapa_wysokosc.place(x=815, y=197, anchor="w")
            tekst_mapa_szerokosc.pack()
            tekst_mapa_szerokosc.place(x=815, y=222, anchor="w")
            wpisz_mapa_wysokosc.pack()
            wpisz_mapa_wysokosc.place(x=993, y=197, anchor="e")
            wpisz_mapa_szerokosc.pack()
            wpisz_mapa_szerokosc.place(x=993, y=222, anchor="e")
            #PRZYCISKI
            przycisk_stworz_mape.pack()
            przycisk_stworz_mape.place(x=905, y=265, anchor="center")
            przycisk_stworz_robota.pack()
            przycisk_stworz_robota.place(x=905, y=312, anchor="center")
            przycisk_usun_robota.pack()
            przycisk_usun_robota.place(x=905, y=359, anchor="center")
            przycisk_stworz_punkt.pack()
            przycisk_stworz_punkt.place(x=905, y=417, anchor="center")
            przycisk_usun_punkt.pack()
            przycisk_usun_punkt.place(x=905, y=487, anchor="center")
            przycisk_stworz_przeszkode_widoczną.pack()
            przycisk_stworz_przeszkode_widoczną.place(x=905, y=557, anchor="center")
            przycisk_stworz_przeszkode_niewidoczną.pack()
            przycisk_stworz_przeszkode_niewidoczną.place(x=905, y=627, anchor="center")
            przycisk_usun_przeszkode.pack()
            przycisk_usun_przeszkode.place(x=905, y=686, anchor="center")
            przycisk_zapisz_mape.pack()
            przycisk_zapisz_mape.place(x=905, y=733, anchor="center")
            przycisk_wyjscie.pack()
            przycisk_wyjscie.place(x=905, y=780, anchor="center")

            okno_tworzenie_mapy.pack()


    def edytuj_mape():
        print("Edytuj mape")

    def usun_mape(okno_stare):
        '''
        Funkcja służąca do usuwania map już wygenerowanych

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
            if czy['wygenerowana_mapa'] == True:
                okno_mapa.delete('all')
                okno_ramka.delete('all')
                czy['wygenerowana_mapa'] = False
            if zbior_dostepnych_map.curselection() != ():
                nr_mapy = zbior_dostepnych_map.curselection()
                nazwa_ppm = zbior_dostepnych_map.get(nr_mapy)
                sciezka_ppm = pathobecny + '\\' + nazwa_ppm
                #sciezka_ppm = "C:\\Users\\User\\OneDrive - Politechnika Wroclawska\\ISA Semestr1\\AO-projekt\\projektpodlogamapy" + '\\' + nazwa_ppm
                nazwa_png = nazwa_ppm.replace(".ppm", ".png")
                sciezka_png = pathobecny + '\\' + nazwa_png
                #sciezka_png = "C:\\Users\\User\\OneDrive - Politechnika Wroclawska\\ISA Semestr1\\AO-projekt\\projektpodlogamapy" + '\\' + nazwa_png
                mapa = odczyt_mapy_z_ppm(sciezka_ppm)
                mapa_png = tk.PhotoImage(file=sciezka_png)
                okno_ramka = tk.Canvas(okno_usuniecia_mapy, height=str(int(mapa['wysokosc'])+10), width=str(int(mapa['szerokosc'])+10), highlightthickness=0)
                okno_ramka.create_rectangle(0, 0, 800+10 ,800+10, fill="black", width=0)
                okno_ramka.pack()
                okno_ramka.place(x=405, y=405, anchor="center")
                okno_mapa = tk.Canvas(okno_usuniecia_mapy, height=mapa['wysokosc'], width=mapa['szerokosc'], highlightthickness=0)
                okno_mapa.create_image(0, 0, anchor="nw", image=mapa_png, tag='mapa')
                okno_mapa.pack()
                okno_mapa.place(x=405, y=405, anchor="center")
                czy['wygenerowana_mapa'] = True

        def usun_zaznaczona_mape():
            '''
            Funkcja służąca usunieciu wybranej mapy

            Funkcja nie zwraca żadnej wartości
            '''
            nr_mapy = zbior_dostepnych_map.curselection()
            nazwa_ppm = zbior_dostepnych_map.get(nr_mapy)
            sciezka_ppm = pathobecny + '\\' + nazwa_ppm
            #sciezka_ppm = "C:\\Users\\User\\OneDrive - Politechnika Wroclawska\\ISA Semestr1\\AO-projekt\\projektpodlogamapy" + '\\' + nazwa_ppm
            nazwa_png = nazwa_ppm.replace(".ppm", ".png")
            sciezka_png= pathobecny + '\\' + nazwa_png
            #sciezka_png = "C:\\Users\\User\\OneDrive - Politechnika Wroclawska\\ISA Semestr1\\AO-projekt\\projektpodlogamapy" + '\\' + nazwa_png
            zbior_dostepnych_map.delete(tk.ANCHOR)
            os.remove(sciezka_ppm)
            os.remove(sciezka_png)

        def powrot_menu_mapy(okno_aktualne, okno_nowe):
            '''
            Funkcja służąca do powrotu do manu mapy

            Funkcja przyjmuje zmienną typu Frame ("okno_aktualne"),
            które jest oknem, które ma zostać dezaktywowane
            Funkcja przyjmuje zmienną typu Frame ("okno_nowe"),
            które jest oknem, które ma zostać aktywowane

            Funkcja nie zwraca żadnej wartości
            '''
            okno_aktualne.pack_forget()
            okno_nowe.pack()
        
        #ZMIENNE
        okno_stare.pack_forget()
        okno_usuniecia_mapy = tk.Frame(okno, height=str(WYSOKOSC_OKNA), width=str(SZEROKOSC_OKNA))
        czy = {'wygenerowana_mapa': False}
    
        #TWORZENIE ELEMENTÓW MAPY
        tekst_dostepne_mapy = tk.Label(okno_usuniecia_mapy, text="Lista dostępnych map:", height=str(0), width=str(0), font = ("timesnewroman", 12))
        rolka_pionowa = tk.Scrollbar(okno_usuniecia_mapy, orient='vertical')
        rolka_pozioma = tk.Scrollbar(okno_usuniecia_mapy, orient='horizontal')
        zbior_dostepnych_map = tk.Listbox(okno_usuniecia_mapy, height=str(35), width=str(24), font = ("timesnewroman", 10), yscrollcommand=rolka_pionowa.set, xscrollcommand=rolka_pozioma.set)
    
        przycisk_usuniecie_mape = tk.Button(okno_usuniecia_mapy, text="Usuń mape", width='15', font = ("timesnewroman", 15), command=lambda: usun_zaznaczona_mape())
        przycisk_podglądaj_mape = tk.Button(okno_usuniecia_mapy, text="Podgląd mapy", width='15', font = ("timesnewroman", 15), command=lambda: podglad_mapy())
        przycisk_wyjscie = tk.Button(okno_usuniecia_mapy, text="Powrót do menu", width='15', font = ("timesnewroman", 15), command=lambda: powrot_menu_mapy(okno_usuniecia_mapy, okno_menu_mapy))
    
        pobierz_mapy()
        
        rolka_pionowa.config(command = zbior_dostepnych_map.yview)
        rolka_pozioma.config(command = zbior_dostepnych_map.xview)
    
        tekst_dostepne_mapy.pack()
        tekst_dostepne_mapy.place(x=810, y=0, anchor="nw")
        zbior_dostepnych_map.pack()
        zbior_dostepnych_map.place(x=810, y=25, anchor="nw")
        rolka_pionowa.pack()
        rolka_pionowa.place(x=1000, y=25, height=513, anchor="ne")
        rolka_pozioma.pack()
        rolka_pozioma.place(x=810, y=625, width=171, anchor="nw")
    
        przycisk_usuniecie_mape.pack
        przycisk_usuniecie_mape.place(x=905, y=680, anchor="center")
        przycisk_podglądaj_mape.pack()
        przycisk_podglądaj_mape.place(x=905, y=730, anchor="center")
        przycisk_wyjscie.pack()
        przycisk_wyjscie.place(x=905, y=780, anchor="center")
    
        okno_usuniecia_mapy.pack()
    
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
            sciezka = "C:\\Users\\User\\OneDrive - Politechnika Wroclawska\\ISA Semestr1\\AO-projekt\\projektpodlogamapy"
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
            if czy['wygenerowana_mapa'] == True:
                okno_mapa.delete('all')
                okno_ramka.delete('all')
                czy['wygenerowana_mapa'] = False
            if zbior_dostepnych_map.curselection() != ():
                nr_mapy = zbior_dostepnych_map.curselection()
                nazwa_ppm = zbior_dostepnych_map.get(nr_mapy)
                sciezka_ppm = pathobecny + '\\' + nazwa_ppm
                #sciezka_ppm = "C:\\Users\\User\\OneDrive - Politechnika Wroclawska\\ISA Semestr1\\AO-projekt\\projektpodlogamapy" + '\\' + nazwa_ppm
                nazwa_png = nazwa_ppm.replace(".ppm", ".png")
                sciezka_png = pathobecny + '\\' + nazwa_png
                #sciezka_png = "C:\\Users\\User\\OneDrive - Politechnika Wroclawska\\ISA Semestr1\\AO-projekt\\projektpodlogamapy" + '\\' + nazwa_png
                mapa = odczyt_mapy_z_ppm(sciezka_ppm)
                mapa_png = tk.PhotoImage(file=sciezka_png)
                okno_ramka = tk.Canvas(okno_przegladu_mapy, height=str(int(mapa['wysokosc'])+10), width=str(int(mapa['szerokosc'])+10), highlightthickness=0)
                okno_ramka.create_rectangle(0, 0, 800+10 ,800+10, fill="black", width=0)
                okno_ramka.pack()
                okno_ramka.place(x=405, y=405, anchor="center")
                okno_mapa = tk.Canvas(okno_przegladu_mapy, height=mapa['wysokosc'], width=mapa['szerokosc'], highlightthickness=0)
                okno_mapa.create_image(0, 0, anchor="nw", image=mapa_png, tag='mapa')
                okno_mapa.pack()
                okno_mapa.place(x=405, y=405, anchor="center")
                czy['wygenerowana_mapa'] = True

        def powrot_menu_mapy(okno_aktualne, okno_nowe):
            '''
            Funkcja służąca do powrotu do manu mapy

            Funkcja przyjmuje zmienną typu Frame ("okno_aktualne"),
            które jest oknem, które ma zostać dezaktywowane
            Funkcja przyjmuje zmienną typu Frame ("okno_nowe"),
            które jest oknem, które ma zostać aktywowane

            Funkcja nie zwraca żadnej wartości
            '''
            okno_aktualne.pack_forget()
            okno_nowe.pack()
        
        #ZMIENNE
        okno_stare.pack_forget()
        okno_przegladu_mapy = tk.Frame(okno, height=str(WYSOKOSC_OKNA), width=str(SZEROKOSC_OKNA))
        czy = {'wygenerowana_mapa': False}
    
        #TWORZENIE ELEMENTÓW MAPY
        tekst_dostepne_mapy = tk.Label(okno_przegladu_mapy, text="Lista dostępnych map:", height=str(0), width=str(0), font = ("timesnewroman", 12))
        rolka_pionowa = tk.Scrollbar(okno_przegladu_mapy, orient='vertical')
        rolka_pozioma = tk.Scrollbar(okno_przegladu_mapy, orient='horizontal')
        zbior_dostepnych_map = tk.Listbox(okno_przegladu_mapy, height=str(38), width=str(24), font = ("timesnewroman", 10), yscrollcommand=rolka_pionowa.set, xscrollcommand=rolka_pozioma.set)

        przycisk_podglądaj_mape = tk.Button(okno_przegladu_mapy, text="Podgląd mapy", width='15', font = ("timesnewroman", 15), command=lambda: podglad_mapy())
        przycisk_wyjscie = tk.Button(okno_przegladu_mapy, text="Powrót do menu", width='15', font = ("timesnewroman", 15), command=lambda: powrot_menu_mapy(okno_przegladu_mapy, okno_menu_mapy))
    
        pobierz_mapy()
        
        rolka_pionowa.config(command = zbior_dostepnych_map.yview)
        rolka_pozioma.config(command = zbior_dostepnych_map.xview)
    
        tekst_dostepne_mapy.pack()
        tekst_dostepne_mapy.place(x=810, y=0, anchor="nw")
        zbior_dostepnych_map.pack()
        zbior_dostepnych_map.place(x=810, y=25, anchor="nw")
        rolka_pionowa.pack()
        rolka_pionowa.place(x=1000, y=25, height=649, anchor="ne")
        rolka_pozioma.pack()
        rolka_pozioma.place(x=810, y=678, width=171, anchor="nw")
    
        przycisk_podglądaj_mape.pack()
        przycisk_podglądaj_mape.place(x=905, y=730, anchor="center")
        przycisk_wyjscie.pack()
        przycisk_wyjscie.place(x=905, y=780, anchor="center")
    
        okno_przegladu_mapy.pack()

    def powrot_menu_glowne(okno_aktualne): #WPROWADZIŁAM ZMIANY
        okno.destroy()
        

    okno_menu_mapy = tk.Frame(okno, height=str(WYSOKOSC_OKNA), width=str(SZEROKOSC_OKNA))
    
    #TWORZENIE NAPISU I PRZYCISKÓW
    tekst_menu_mapy = tk.Label(okno_menu_mapy, text="MENU MAPY", height=str(0), width=str(0), font = ("timesnewroman", 20, "bold"))
    przycisk_stworz_mape = tk.Button(okno_menu_mapy, text="Stwórz mapę", width='30', height='3', font = ("timesnewroman", 15, 'bold'), command=lambda: stworz_mape(okno_menu_mapy))
    przycisk_usun_mape = tk.Button(okno_menu_mapy, text="Usun mapę", width='30', height='3', font = ("timesnewroman", 15, 'bold'), command=lambda: usun_mape(okno_menu_mapy))
    przycisk_przeglad_map = tk.Button(okno_menu_mapy, text="Przeglad map", width='30', height='3', font = ("timesnewroman", 15, 'bold'), command=lambda: przeglad_map(okno_menu_mapy))
    przycisk_menu_glowne = tk.Button(okno_menu_mapy, text="Powrot do menu głównego", width='30', height='3', font = ("timesnewroman", 15, 'bold'), command=lambda: powrot_menu_glowne(okno_menu_mapy))

    tekst_menu_mapy.pack()
    tekst_menu_mapy.place(x=SZEROKOSC_OKNA/2, y=WYSOKOSC_OKNA/2 - 280, anchor="center")
    przycisk_stworz_mape.pack()
    przycisk_stworz_mape.place(x=SZEROKOSC_OKNA/2, y=WYSOKOSC_OKNA/2 - 200, anchor="center")
    przycisk_usun_mape.pack()
    przycisk_usun_mape.place(x=SZEROKOSC_OKNA/2, y=WYSOKOSC_OKNA/2 - 100, anchor="center")
    przycisk_przeglad_map.pack()
    przycisk_przeglad_map.place(x=SZEROKOSC_OKNA/2, y=WYSOKOSC_OKNA/2, anchor="center")
    przycisk_menu_glowne.pack()
    przycisk_menu_glowne.place(x=SZEROKOSC_OKNA/2, y=WYSOKOSC_OKNA/2 + 100, anchor="center")

    okno_menu_mapy.pack()

