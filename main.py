# Standard library imports
import sys
import os
import math
import time
import threading
import datetime
import pickle
import pathlib
from functools import partial
from os import execv, path

# Third-party libraries
import numpy as np
import cv2
from numpy import mean
from PIL import Image, ImageTk
from matplotlib import pyplot as plt
from inputs import get_gamepad
import serial

# CustomTkinter and Tkinter imports
import customtkinter as ctk
from tkinter import messagebox, END, filedialog as fd

# Additional imports
from multiprocessing.resource_sharer import stop
from pygrabber.dshow_graph import FilterGraph

# Application code #
class RobotArmApp:
    def __init__(self):
        # Main window setup
        self.root = ctk.CTk()
        self.root.title("Robot Arm Software Ver 6.0")
        self.root.iconbitmap(os.path.join('assets', 'EE.ico'))
        self.root.resizable(width=False, height=False)
        self.root.geometry('1536x792+0+0')

        # Define on_closing as an instance method
        def on_closing():
            # Handle program exit with cleanup
            if messagebox.askokcancel("Close Program", "Do you want to quit?"):
                try:
                    command = "CL"
                    if self.ser:
                        self.ser.write(command.encode())
                except Exception as e:
                    print("Error in closing command:", e)
                finally:
                    if self.ser:
                        self.ser.close()
                    self.root.destroy()

        self.root.wm_protocol("WM_DELETE_WINDOW", on_closing)

        # GUI state variables
        self.JogStepsStat = ctk.IntVar()
        self.J1OpenLoopStat = ctk.IntVar()
        self.J2OpenLoopStat = ctk.IntVar()
        self.J3OpenLoopStat = ctk.IntVar()
        self.J4OpenLoopStat = ctk.IntVar()
        self.J5OpenLoopStat = ctk.IntVar()
        self.J6OpenLoopStat = ctk.IntVar()
        self.DisableWristRot = ctk.IntVar()
        self.xboxUse = ctk.IntVar()
        self.curTheme = 1
        self.J1CalStat = ctk.IntVar()
        self.J2CalStat = ctk.IntVar()
        self.J3CalStat = ctk.IntVar()
        self.J4CalStat = ctk.IntVar()
        self.J5CalStat = ctk.IntVar()
        self.J6CalStat = ctk.IntVar()
        self.J7CalStat = ctk.IntVar()
        self.J8CalStat = ctk.IntVar()
        self.J9CalStat = ctk.IntVar()
        self.J1CalStat2 = ctk.IntVar()
        self.J2CalStat2 = ctk.IntVar()
        self.J3CalStat2 = ctk.IntVar()
        self.J4CalStat2 = ctk.IntVar()
        self.J5CalStat2 = ctk.IntVar()
        self.J6CalStat2 = ctk.IntVar()
        self.J7CalStat2 = ctk.IntVar()
        self.J8CalStat2 = ctk.IntVar()
        self.J9CalStat2 = ctk.IntVar()
        self.IncJogStat = ctk.IntVar()
        self.fullRot = ctk.IntVar()
        self.pick180 = ctk.IntVar()
        self.pickClosest = ctk.IntVar()
        self.autoBG = ctk.IntVar()

        # Axis limits
        self.J1PosLim = 170
        self.J1NegLim = 170
        self.J2PosLim = 90
        self.J2NegLim = 42
        self.J3PosLim = 52
        self.J3NegLim = 89
        self.J4PosLim = 165
        self.J4NegLim = 165
        self.J5PosLim = 105
        self.J5NegLim = 105
        self.J6PosLim = 155
        self.J6NegLim = 155
        self.J7PosLim = 500
        self.J7NegLim = 0
        self.J8PosLim = 500
        self.J8NegLim = 0
        self.J9PosLim = 500
        self.J9NegLim = 0

        # Other key states
        self.cropping = False
        self.cam_on = False
        self.cap = None
        self.estopActive = False
        self.posOutreach = False
        self.SplineTrue = False
        self.gcodeSpeed = "10"
        self.inchTrue = False
        self.moveInProc = 0
        
        # Initialize serial connection
        self.ser = None
        self.ser2 = None
        self.ser3 = None

        # Define Tabs
        self.nb = ctk.CTkTabview(self.root, width=1536, height=792)
        self.nb.place(x=0, y=0)

        # Add tabs with frames properly placed
        self.tab1_name = self.nb.add("Main Controls")
        self.tab1 = ctk.CTkFrame(self.tab1_name, width=1536, height=792, fg_color="transparent")
        self.tab1.place(x=0, y=0)  # Ensure the frame fills the tab

        self.tab2_name = self.nb.add("Config Settings")
        self.tab2 = ctk.CTkFrame(self.tab2_name, width=1536, height=792, fg_color="transparent")
        self.tab2.place(x=0, y=0)

        self.tab3_name = self.nb.add("Kinematics")
        self.tab3 = ctk.CTkFrame(self.tab3_name, width=1536, height=792, fg_color="transparent")
        self.tab3.place(x=0, y=0)

        self.tab4_name = self.nb.add("Inputs Outputs")
        self.tab4 = ctk.CTkFrame(self.tab4_name, width=1536, height=792, fg_color="transparent")
        self.tab4.place(x=0, y=0)

        self.tab5_name = self.nb.add("Registers")
        self.tab5 = ctk.CTkFrame(self.tab5_name, width=1536, height=792, fg_color="transparent")
        self.tab5.place(x=0, y=0)

        self.tab6_name = self.nb.add("Vision")
        self.tab6 = ctk.CTkFrame(self.tab6_name, width=1536, height=792, fg_color="transparent")
        self.tab6.place(x=0, y=0)

        self.tab7_name = self.nb.add("G-Code")
        self.tab7 = ctk.CTkFrame(self.tab7_name, width=1536, height=792, fg_color="transparent")
        self.tab7.place(x=0, y=0)

        self.tab8_name = self.nb.add("Log")
        self.tab8 = ctk.CTkFrame(self.tab8_name, width=1536, height=792, fg_color="transparent")
        self.tab8.place(x=0, y=0)

        # Application styling defs

        ## TAB 1 LABELS ##

        self.CartjogFrame = ctk.CTkFrame(self.tab1, width=1536, height=792)
        self.CartjogFrame.place(x=330, y=0)

        self.curRowLab = ctk.CTkLabel(self.tab1, text="Current Row:")
        self.curRowLab.place(x=98, y=120)

        self.almStatusLab = ctk.CTkLabel(
            self.tab1, text="SYSTEM READY - NO ACTIVE ALARMS", text_color="green"
        )
        self.almStatusLab.place(x=25, y=12)

        self.runStatusLab = ctk.CTkLabel(self.tab1, text="PROGRAM STOPPED")
        self.runStatusLab.place(x=20, y=150)

        self.ProgLab = ctk.CTkLabel(self.tab1, text="Program:")
        self.ProgLab.place(x=10, y=45)

        self.jogIncrementLab = ctk.CTkLabel(self.tab1, text="Increment Value:")
        # self.jogIncrementLab.place(x=370, y=45)

        self.speedLab = ctk.CTkLabel(self.tab1, text="Speed")
        self.speedLab.place(x=300, y=83)

        self.ACCLab = ctk.CTkLabel(self.tab1, text="Acceleration %")
        self.ACCLab.place(x=300, y=103)

        self.DECLab = ctk.CTkLabel(self.tab1, text="Deceleration %")
        self.DECLab.place(x=300, y=123)

        self.RampLab = ctk.CTkLabel(self.tab1, text="Ramp %")
        self.RampLab.place(x=300, y=143)

        self.RoundLab = ctk.CTkLabel(self.tab1, text="Rounding mm")
        self.RoundLab.place(x=525, y=82)

        """ Cartesian Jog Labels """

        self.XLab = ctk.CTkLabel(self.CartjogFrame, font=("Arial", 18), text=" X")
        self.XLab.place(x=660, y=162)

        self.YLab = ctk.CTkLabel(self.CartjogFrame, font=("Arial", 18), text=" Y")
        self.YLab.place(x=750, y=162)

        self.ZLab = ctk.CTkLabel(self.CartjogFrame, font=("Arial", 18), text=" Z")
        self.ZLab.place(x=840, y=162)

        self.yLab = ctk.CTkLabel(self.CartjogFrame, font=("Arial", 18), text="Rz")
        self.yLab.place(x=930, y=162)

        self.pLab = ctk.CTkLabel(self.CartjogFrame, font=("Arial", 18), text="Ry")
        self.pLab.place(x=1020, y=162)

        self.rLab = ctk.CTkLabel(self.CartjogFrame, font=("Arial", 18), text="Rx")
        self.rLab.place(x=1110, y=162)

        self.TXLab = ctk.CTkLabel(self.CartjogFrame, font=("Arial", 18), text="Tx")
        self.TXLab.place(x=660, y=265)

        self.TYLab = ctk.CTkLabel(self.CartjogFrame, font=("Arial", 18), text="Ty")
        self.TYLab.place(x=750, y=265)

        self.TZLab = ctk.CTkLabel(self.CartjogFrame, font=("Arial", 18), text="Tz")
        self.TZLab.place(x=840, y=265)

        self.TyLab = ctk.CTkLabel(self.CartjogFrame, font=("Arial", 18), text="Trz")
        self.TyLab.place(x=930, y=265)

        self.TpLab = ctk.CTkLabel(self.CartjogFrame, font=("Arial", 18), text="Try")
        self.TpLab.place(x=1020, y=265)

        self.J7Lab = ctk.CTkLabel(self.CartjogFrame, font=("Arial", 18), text="Trx")
        self.J7Lab.place(x=1110, y=265)

        """ J1 Joint Controls """

        self.J1jogFrame = ctk.CTkFrame(self.tab1, width=340, height=40)
        self.J1jogFrame.place(x=810, y=10)

        self.J1Lab = ctk.CTkLabel(self.J1jogFrame, font=("Arial", 18), text="J1")
        self.J1Lab.place(x=5, y=5)

        self.J1curAngEntryField = ctk.CTkEntry(self.J1jogFrame, width=50, justify="center")
        self.J1curAngEntryField.place(x=35, y=9)

        def SelJ1jogNeg(event):
            IncJogStatVal = int(self.IncJogStat.get())
            if IncJogStatVal == 1:
                self.J1jogNeg(float(self.incrementEntryField.get()))
            else:
                self.LiveJointJog(10)

        self.J1jogNegBut = ctk.CTkButton(self.J1jogFrame, text="-", width=30, height=25)
        self.J1jogNegBut.bind("<ButtonPress>", SelJ1jogNeg)
        self.J1jogNegBut.bind("<ButtonRelease>", self.StopJog)
        self.J1jogNegBut.place(x=77, y=7)

        def SelJ1jogPos(event):
            IncJogStatVal = int(self.IncJogStat.get())
            if IncJogStatVal == 1:
                self.J1jogPos(float(self.incrementEntryField.get()))
            else:
                self.LiveJointJog(11)

        self.J1jogPosBut = ctk.CTkButton(self.J1jogFrame, text="+", width=30, height=25)
        self.J1jogPosBut.bind("<ButtonPress>", SelJ1jogPos)
        self.J1jogPosBut.bind("<ButtonRelease>", self.StopJog)
        self.J1jogPosBut.place(x=300, y=7)

        self.J1negLimLab = ctk.CTkLabel(self.J1jogFrame, font=("Arial", 8), text=str(-self.J1NegLim))
        self.J1negLimLab.place(x=115, y=25)

        self.J1posLimLab = ctk.CTkLabel(self.J1jogFrame, font=("Arial", 8), text=str(self.J1PosLim))
        self.J1posLimLab.place(x=270, y=25)

        self.J1slidelabel = ctk.CTkLabel(self.J1jogFrame)
        self.J1slidelabel.place(x=190, y=25)

        def J1sliderUpdate(value):
            self.J1slidelabel.configure(text=round(float(self.J1jogslide.get()), 2))

        def J1sliderExecute(event):
            J1delta = float(self.J1jogslide.get()) - float(self.J1curAngEntryField.get())
            if J1delta < 0:
                self.J1jogNeg(abs(J1delta))
            else:
                self.J1jogPos(abs(J1delta))

        self.J1jogslide = ctk.CTkSlider(
            self.J1jogFrame, from_=-self.J1NegLim, to=self.J1PosLim, orientation="horizontal", command=J1sliderUpdate, width=180
        )
        self.J1jogslide.bind("<ButtonRelease-1>", J1sliderExecute)
        self.J1jogslide.place(x=115, y=7)

        """ J2 Joint Controls """

        self.J2jogFrame = ctk.CTkFrame(self.tab1, width=340, height=40)
        self.J2jogFrame.place(x=810, y=55)

        self.J2Lab = ctk.CTkLabel(self.J2jogFrame, font=("Arial", 18), text="J2")
        self.J2Lab.place(x=5, y=5)

        self.J2curAngEntryField = ctk.CTkEntry(self.J2jogFrame, width=50, justify="center")
        self.J2curAngEntryField.place(x=35, y=9)

        def SelJ2jogNeg(event):
            IncJogStatVal = int(self.IncJogStat.get())
            if IncJogStatVal == 1:
                self.J2jogNeg(float(self.incrementEntryField.get()))
            else:
                self.LiveJointJog(20)

        self.J2jogNegBut = ctk.CTkButton(self.J2jogFrame, text="-", width=30, height=25)
        self.J2jogNegBut.bind("<ButtonPress>", SelJ2jogNeg)
        self.J2jogNegBut.bind("<ButtonRelease>", self.StopJog)
        self.J2jogNegBut.place(x=77, y=7)

        def SelJ2jogPos(event):
            IncJogStatVal = int(self.IncJogStat.get())
            if IncJogStatVal == 1:
                self.J2jogPos(float(self.incrementEntryField.get()))
            else:
                self.LiveJointJog(21)

        self.J2jogPosBut = ctk.CTkButton(self.J2jogFrame, text="+", width=30, height=25)
        self.J2jogPosBut.bind("<ButtonPress>", SelJ2jogPos)
        self.J2jogPosBut.bind("<ButtonRelease>", self.StopJog)
        self.J2jogPosBut.place(x=300, y=7)

        self.J2negLimLab = ctk.CTkLabel(self.J2jogFrame, font=("Arial", 8), text=str(-self.J2NegLim))
        self.J2negLimLab.place(x=115, y=25)

        self.J2posLimLab = ctk.CTkLabel(self.J2jogFrame, font=("Arial", 8), text=str(self.J2PosLim))
        self.J2posLimLab.place(x=270, y=25)

        self.J2slidelabel = ctk.CTkLabel(self.J2jogFrame)
        self.J2slidelabel.place(x=190, y=25)

        def J2sliderUpdate(value):
            self.J2slidelabel.configure(text=round(float(self.J2jogslide.get()), 2))

        def J2sliderExecute(event):
            J2delta = float(self.J2jogslide.get()) - float(self.J2curAngEntryField.get())
            if J2delta < 0:
                self.J2jogNeg(abs(J2delta))
            else:
                self.J2jogPos(abs(J2delta))

        self.J2jogslide = ctk.CTkSlider(
            self.J2jogFrame, from_=-self.J2NegLim, to=self.J2PosLim, orientation='horizontal', command=J2sliderUpdate, width=180
        )
        self.J2jogslide.bind("<ButtonRelease-1>", J2sliderExecute)
        self.J2jogslide.place(x=115, y=7)

        """ J3 Joint Controls """

        self.J3jogFrame = ctk.CTkFrame(self.tab1, width=340, height=40)
        self.J3jogFrame.place(x=810, y=100)

        self.J3Lab = ctk.CTkLabel(self.J3jogFrame, font=("Arial", 18), text="J3")
        self.J3Lab.place(x=5, y=5)

        self.J3curAngEntryField = ctk.CTkEntry(self.J3jogFrame, width=50, justify="center")
        self.J3curAngEntryField.place(x=35, y=9)

        def SelJ3jogNeg(event):
            IncJogStatVal = int(self.IncJogStat.get())
            if IncJogStatVal == 1:
                self.J3jogNeg(float(self.incrementEntryField.get()))
            else:
                self.LiveJointJog(30)

        self.J3jogNegBut = ctk.CTkButton(self.J3jogFrame, text="-", width=30, height=25)
        self.J3jogNegBut.bind("<ButtonPress>", SelJ3jogNeg)
        self.J3jogNegBut.bind("<ButtonRelease>", self.StopJog)
        self.J3jogNegBut.place(x=77, y=7)

        def SelJ3jogPos(event):
            IncJogStatVal = int(self.IncJogStat.get())
            if IncJogStatVal == 1:
                self.J3jogPos(float(self.incrementEntryField.get()))
            else:
                self.LiveJointJog(31)

        self.J3jogPosBut = ctk.CTkButton(self.J3jogFrame, text="+", width=30, height=25)
        self.J3jogPosBut.bind("<ButtonPress>", SelJ3jogPos)
        self.J3jogPosBut.bind("<ButtonRelease>", self.StopJog)
        self.J3jogPosBut.place(x=300, y=7)

        self.J3negLimLab = ctk.CTkLabel(self.J3jogFrame, font=("Arial", 8), text=str(-self.J3NegLim))
        self.J3negLimLab.place(x=115, y=25)

        self.J3posLimLab = ctk.CTkLabel(self.J3jogFrame, font=("Arial", 8), text=str(self.J3PosLim))
        self.J3posLimLab.place(x=270, y=25)

        self.J3slidelabel = ctk.CTkLabel(self.J3jogFrame)
        self.J3slidelabel.place(x=190, y=25)

        def J3sliderUpdate(value):
            self.J3slidelabel.configure(text=round(float(self.J3jogslide.get()), 2))

        def J3sliderExecute(event):
            J3delta = float(self.J3jogslide.get()) - float(self.J3curAngEntryField.get())
            if J3delta < 0:
                self.J3jogNeg(abs(J3delta))
            else:
                self.J3jogPos(abs(J3delta))

        self.J3jogslide = ctk.CTkSlider(
            self.J3jogFrame, from_=-self.J3NegLim, to=self.J3PosLim, orientation="horizontal", command=J3sliderUpdate, width=180
        )
        self.J3jogslide.bind("<ButtonRelease-1>", J3sliderExecute)
        self.J3jogslide.place(x=115, y=7)

        """ J4 Joint Controls """

        self.J4jogFrame = ctk.CTkFrame(self.tab1, width=340, height=40)
        self.J4jogFrame.place(x=1160, y=10)

        self.J4Lab = ctk.CTkLabel(self.J4jogFrame, font=("Arial", 18), text="J4")
        self.J4Lab.place(x=5, y=5)

        self.J4curAngEntryField = ctk.CTkEntry(self.J4jogFrame, width=50, justify="center")
        self.J4curAngEntryField.place(x=35, y=9)

        # Jog Negative Button
        def SelJ4jogNeg(event):
            IncJogStatVal = int(self.IncJogStat.get())
            if IncJogStatVal == 1:
                self.J4jogNeg(float(self.incrementEntryField.get()))
            else:
                self.LiveJointJog(40)

        self.J4jogNegBut = ctk.CTkButton(self.J4jogFrame, text="-", width=30, height=25)
        self.J4jogNegBut.bind("<ButtonPress>", SelJ4jogNeg)
        self.J4jogNegBut.bind("<ButtonRelease>", self.StopJog)
        self.J4jogNegBut.place(x=77, y=7)

        # Jog Positive Button
        def SelJ4jogPos(event):
            IncJogStatVal = int(self.IncJogStat.get())
            if IncJogStatVal == 1:
                self.J4jogPos(float(self.incrementEntryField.get()))
            else:
                self.LiveJointJog(41)

        self.J4jogPosBut = ctk.CTkButton(self.J4jogFrame, text="+", width=30, height=25)
        self.J4jogPosBut.bind("<ButtonPress>", SelJ4jogPos)
        self.J4jogPosBut.bind("<ButtonRelease>", self.StopJog)
        self.J4jogPosBut.place(x=300, y=7)

        self.J4negLimLab = ctk.CTkLabel(self.J4jogFrame, font=("Arial", 8), text=str(-self.J4NegLim))
        self.J4negLimLab.place(x=115, y=25)

        self.J4posLimLab = ctk.CTkLabel(self.J4jogFrame, font=("Arial", 8), text=str(self.J4PosLim))
        self.J4posLimLab.place(x=270, y=25)

        self.J4slidelabel = ctk.CTkLabel(self.J4jogFrame)
        self.J4slidelabel.place(x=190, y=25)

        # Slider Update Function
        def J4sliderUpdate(value):
            self.J4slidelabel.configure(text=round(float(self.J4jogslide.get()), 2))

        # Slider Execute Function
        def J4sliderExecute(event):
            J4delta = float(self.J4jogslide.get()) - float(self.J4curAngEntryField.get())
            if J4delta < 0:
                self.J4jogNeg(abs(J4delta))
            else:
                self.J4jogPos(abs(J4delta))

        self.J4jogslide = ctk.CTkSlider(
            self.J4jogFrame, from_=-self.J4NegLim, to=self.J4PosLim, orientation="horizontal", command=J4sliderUpdate, width=180
        )
        self.J4jogslide.bind("<ButtonRelease-1>", J4sliderExecute)
        self.J4jogslide.place(x=115, y=7)

        """ J5 Joint Controls """

        self.J5jogFrame = ctk.CTkFrame(self.tab1, width=340, height=40)
        self.J5jogFrame.place(x=1160, y=55)

        self.J5Lab = ctk.CTkLabel(self.J5jogFrame, font=("Arial", 18), text="J5")
        self.J5Lab.place(x=5, y=5)

        self.J5curAngEntryField = ctk.CTkEntry(self.J5jogFrame, width=50, justify="center")
        self.J5curAngEntryField.place(x=35, y=9)

        # Jog Negative Button
        def SelJ5jogNeg(event):
            IncJogStatVal = int(self.IncJogStat.get())
            if IncJogStatVal == 1:
                self.J5jogNeg(float(self.incrementEntryField.get()))
            else:
                self.LiveJointJog(50)

        self.J5jogNegBut = ctk.CTkButton(self.J5jogFrame, text="-", width=30, height=25)
        self.J5jogNegBut.bind("<ButtonPress>", SelJ5jogNeg)
        self.J5jogNegBut.bind("<ButtonRelease>", self.StopJog)
        self.J5jogNegBut.place(x=77, y=7)

        # Jog Positive Button
        def SelJ5jogPos(event):
            IncJogStatVal = int(self.IncJogStat.get())
            if IncJogStatVal == 1:
                self.J5jogPos(float(self.incrementEntryField.get()))
            else:
                self.LiveJointJog(51)

        self.J5jogPosBut = ctk.CTkButton(self.J5jogFrame, text="+", width=30, height=25)
        self.J5jogPosBut.bind("<ButtonPress>", SelJ5jogPos)
        self.J5jogPosBut.bind("<ButtonRelease>", self.StopJog)
        self.J5jogPosBut.place(x=300, y=7)

        self.J5negLimLab = ctk.CTkLabel(self.J5jogFrame, font=("Arial", 8), text=str(-self.J5NegLim))
        self.J5negLimLab.place(x=115, y=25)

        self.J5posLimLab = ctk.CTkLabel(self.J5jogFrame, font=("Arial", 8), text=str(self.J5PosLim))
        self.J5posLimLab.place(x=270, y=25)

        self.J5slidelabel = ctk.CTkLabel(self.J5jogFrame)
        self.J5slidelabel.place(x=190, y=25)

        # Slider Update Function
        def J5sliderUpdate(value):
            self.J5slidelabel.configure(text=round(float(self.J5jogslide.get()), 2))

        # Slider Execute Function
        def J5sliderExecute(event):
            J5delta = float(self.J5jogslide.get()) - float(self.J5curAngEntryField.get())
            if J5delta < 0:
                self.J5jogNeg(abs(J5delta))
            else:
                self.J5jogPos(abs(J5delta))

        self.J5jogslide = ctk.CTkSlider(
            self.J5jogFrame, from_=-self.J5NegLim, to=self.J5PosLim, orientation="horizontal", command=J5sliderUpdate, width=180
        )
        self.J5jogslide.bind("<ButtonRelease-1>", J5sliderExecute)
        self.J5jogslide.place(x=115, y=7)

        """ J6 Joint Controls """

        self.J6jogFrame = ctk.CTkFrame(self.tab1, width=340, height=40)
        self.J6jogFrame.place(x=1160, y=100)

        self.J6Lab = ctk.CTkLabel(self.J6jogFrame, font=("Arial", 18), text="J6")
        self.J6Lab.place(x=5, y=5)

        self.J6curAngEntryField = ctk.CTkEntry(self.J6jogFrame, width=50, justify="center")
        self.J6curAngEntryField.place(x=35, y=9)

        # Jog Negative Button
        def SelJ6jogNeg(event):
            IncJogStatVal = int(self.IncJogStat.get())
            if IncJogStatVal == 1:
                self.J6jogNeg(float(self.incrementEntryField.get()))
            else:
                self.LiveJointJog(60)

        self.J6jogNegBut = ctk.CTkButton(self.J6jogFrame, text="-", width=30, height=25)
        self.J6jogNegBut.bind("<ButtonPress>", SelJ6jogNeg)
        self.J6jogNegBut.bind("<ButtonRelease>", self.StopJog)
        self.J6jogNegBut.place(x=77, y=7)

        # Jog Positive Button
        def SelJ6jogPos(event):
            IncJogStatVal = int(self.IncJogStat.get())
            if IncJogStatVal == 1:
                self.J6jogPos(float(self.incrementEntryField.get()))
            else:
                self.LiveJointJog(61)

        self.J6jogPosBut = ctk.CTkButton(self.J6jogFrame, text="+", width=30, height=25)
        self.J6jogPosBut.bind("<ButtonPress>", SelJ6jogPos)
        self.J6jogPosBut.bind("<ButtonRelease>", self.StopJog)
        self.J6jogPosBut.place(x=300, y=7)

        self.J6negLimLab = ctk.CTkLabel(self.J6jogFrame, font=("Arial", 8), text=str(-self.J6NegLim))
        self.J6negLimLab.place(x=115, y=25)

        self.J6posLimLab = ctk.CTkLabel(self.J6jogFrame, font=("Arial", 8), text=str(self.J6PosLim))
        self.J6posLimLab.place(x=270, y=25)

        self.J6slidelabel = ctk.CTkLabel(self.J6jogFrame)
        self.J6slidelabel.place(x=190, y=25)

        # Slider Update Function
        def J6sliderUpdate(value):
            self.J6slidelabel.configure(text=round(float(self.J6jogslide.get()), 2))

        # Slider Execute Function
        def J6sliderExecute(event):
            J6delta = float(self.J6jogslide.get()) - float(self.J6curAngEntryField.get())
            if J6delta < 0:
                self.J6jogNeg(abs(J6delta))
            else:
                self.J6jogPos(abs(J6delta))

        self.J6jogslide = ctk.CTkSlider(
            self.J6jogFrame, from_=-self.J6NegLim, to=self.J6PosLim, orientation="horizontal", command=J6sliderUpdate, width=180
        )
        self.J6jogslide.bind("<ButtonRelease-1>", J6sliderExecute)
        self.J6jogslide.place(x=115, y=7)

        """ J7 Joint Controls """

        self.J7jogFrame = ctk.CTkFrame(self.tab1, width=145, height=100, corner_radius=5)
        self.J7jogFrame.place(x=1340, y=420)

        self.J7Lab = ctk.CTkLabel(self.J7jogFrame, font=("Arial", 14), text="7th Axis")
        self.J7Lab.place(x=15, y=5)

        self.J7curAngEntryField = ctk.CTkEntry(self.J7jogFrame, width=50, justify="center")
        self.J7curAngEntryField.place(x=75, y=9)

        # Jog Negative Button
        def SelJ7jogNeg(event):
            IncJogStatVal = int(self.IncJogStat.get())
            if IncJogStatVal == 1:
                self.J7jogNeg(float(self.incrementEntryField.get()))
            else:
                self.LiveJointJog(70)

        self.J7jogNegBut = ctk.CTkButton(self.J7jogFrame, text="-", width=30, height=25)
        self.J7jogNegBut.bind("<ButtonPress>", SelJ7jogNeg)
        self.J7jogNegBut.bind("<ButtonRelease>", self.StopJog)
        self.J7jogNegBut.place(x=10, y=65)

        # Jog Positive Button
        def SelJ7jogPos(event):
            IncJogStatVal = int(self.IncJogStat.get())
            if IncJogStatVal == 1:
                self.J7jogPos(float(self.incrementEntryField.get()))
            else:
                self.LiveJointJog(71)

        self.J7jogPosBut = ctk.CTkButton(self.J7jogFrame, text="+", width=30, height=25)
        self.J7jogPosBut.bind("<ButtonPress>", SelJ7jogPos)
        self.J7jogPosBut.bind("<ButtonRelease>", self.StopJog)
        self.J7jogPosBut.place(x=105, y=65)

        self.J7negLimLab = ctk.CTkLabel(self.J7jogFrame, font=("Arial", 8), text=str(-self.J7NegLim))
        self.J7negLimLab.place(x=10, y=30)

        self.J7posLimLab = ctk.CTkLabel(self.J7jogFrame, font=("Arial", 8), text=str(self.J7PosLim))
        self.J7posLimLab.place(x=110, y=30)

        self.J7slideLimLab = ctk.CTkLabel(self.J7jogFrame)
        self.J7slideLimLab.place(x=60, y=70)

        # Slider Update Function
        def J7sliderUpdate(value):
            self.J7slideLimLab.configure(text=round(float(self.J7jogslide.get()), 2))

        # Slider Execute Function
        def J7sliderExecute(event):
            J7delta = float(self.J7jogslide.get()) - float(self.J7curAngEntryField.get())
            if J7delta < 0:
                self.J7jogNeg(abs(J7delta))
            else:
                self.J7jogPos(abs(J7delta))

        self.J7jogslide = ctk.CTkSlider(
            self.J7jogFrame, from_=-self.J7NegLim, to=self.J7PosLim, orientation="horizontal", command=J7sliderUpdate, width=125
        )
        self.J7jogslide.bind("<ButtonRelease-1>", J7sliderExecute)
        self.J7jogslide.place(x=10, y=43)

        """ J8 Joint Controls """

        self.J8jogFrame = ctk.CTkFrame(self.tab1, width=145, height=100, corner_radius=5)
        self.J8jogFrame.place(x=1340, y=530)

        self.J8Lab = ctk.CTkLabel(self.J8jogFrame, font=("Arial", 14), text="8th Axis")
        self.J8Lab.place(x=15, y=5)

        self.J8curAngEntryField = ctk.CTkEntry(self.J8jogFrame, width=50, justify="center")
        self.J8curAngEntryField.place(x=75, y=9)

        # Jog Negative Button
        def SelJ8jogNeg(event):
            IncJogStatVal = int(self.IncJogStat.get())
            if IncJogStatVal == 1:
                self.J8jogNeg(float(self.incrementEntryField.get()))
            else:
                self.LiveJointJog(80)

        self.J8jogNegBut = ctk.CTkButton(self.J8jogFrame, text="-", width=30, height=25)
        self.J8jogNegBut.bind("<ButtonPress>", SelJ8jogNeg)
        self.J8jogNegBut.bind("<ButtonRelease>", self.StopJog)
        self.J8jogNegBut.place(x=10, y=65)

        # Jog Positive Button
        def SelJ8jogPos(event):
            IncJogStatVal = int(self.IncJogStat.get())
            if IncJogStatVal == 1:
                self.J8jogPos(float(self.incrementEntryField.get()))
            else:
                self.LiveJointJog(81)

        self.J8jogPosBut = ctk.CTkButton(self.J8jogFrame, text="+", width=30, height=25)
        self.J8jogPosBut.bind("<ButtonPress>", SelJ8jogPos)
        self.J8jogPosBut.bind("<ButtonRelease>", self.StopJog)
        self.J8jogPosBut.place(x=105, y=65)

        self.J8negLimLab = ctk.CTkLabel(self.J8jogFrame, font=("Arial", 8), text=str(-self.J8NegLim))
        self.J8negLimLab.place(x=10, y=30)

        self.J8posLimLab = ctk.CTkLabel(self.J8jogFrame, font=("Arial", 8), text=str(self.J8PosLim))
        self.J8posLimLab.place(x=110, y=30)

        self.J8slideLimLab = ctk.CTkLabel(self.J8jogFrame)
        self.J8slideLimLab.place(x=60, y=70)

        # Slider Update Function
        def J8sliderUpdate(value):
            self.J8slideLimLab.configure(text=round(float(self.J8jogslide.get()), 2))

        # Slider Execute Function
        def J8sliderExecute(event):
            J8delta = float(self.J8jogslide.get()) - float(self.J8curAngEntryField.get())
            if J8delta < 0:
                self.J8jogNeg(abs(J8delta))
            else:
                self.J8jogPos(abs(J8delta))

        self.J8jogslide = ctk.CTkSlider(
            self.J8jogFrame, from_=-self.J8NegLim, to=self.J8PosLim, orientation="horizontal", command=J8sliderUpdate, width=125
        )
        self.J8jogslide.bind("<ButtonRelease-1>", J8sliderExecute)
        self.J8jogslide.place(x=10, y=43)

        """ J9 Joint Controls """

        self.J9jogFrame = ctk.CTkFrame(self.tab1, width=145, height=100, corner_radius=5)
        self.J9jogFrame.place(x=1340, y=640)

        self.J9Lab = ctk.CTkLabel(self.J9jogFrame, font=("Arial", 14), text="9th Axis")
        self.J9Lab.place(x=15, y=5)

        self.J9curAngEntryField = ctk.CTkEntry(self.J9jogFrame, width=50, justify="center")
        self.J9curAngEntryField.place(x=75, y=9)

        # Jog Negative Button
        def SelJ9jogNeg(event):
            IncJogStatVal = int(self.IncJogStat.get())
            if IncJogStatVal == 1:
                self.J9jogNeg(float(self.incrementEntryField.get()))
            else:
                self.LiveJointJog(90)

        self.J9jogNegBut = ctk.CTkButton(self.J9jogFrame, text="-", width=30, height=25)
        self.J9jogNegBut.bind("<ButtonPress>", SelJ9jogNeg)
        self.J9jogNegBut.bind("<ButtonRelease>", self.StopJog)
        self.J9jogNegBut.place(x=10, y=65)

        # Jog Positive Button
        def SelJ9jogPos(event):
            IncJogStatVal = int(self.IncJogStat.get())
            if IncJogStatVal == 1:
                self.J9jogPos(float(self.incrementEntryField.get()))
            else:
                self.LiveJointJog(91)

        self.J9jogPosBut = ctk.CTkButton(self.J9jogFrame, text="+", width=30, height=25)
        self.J9jogPosBut.bind("<ButtonPress>", SelJ9jogPos)
        self.J9jogPosBut.bind("<ButtonRelease>", self.StopJog)
        self.J9jogPosBut.place(x=105, y=65)

        self.J9negLimLab = ctk.CTkLabel(self.J9jogFrame, font=("Arial", 8), text=str(-self.J9NegLim))
        self.J9negLimLab.place(x=10, y=30)

        self.J9posLimLab = ctk.CTkLabel(self.J9jogFrame, font=("Arial", 8), text=str(self.J9PosLim))
        self.J9posLimLab.place(x=110, y=30)

        self.J9slideLimLab = ctk.CTkLabel(self.J9jogFrame)
        self.J9slideLimLab.place(x=60, y=70)

        # Slider Update Function
        def J9sliderUpdate(value):
            self.J9slideLimLab.configure(text=round(float(self.J9jogslide.get()), 2))

        # Slider Execute Function
        def J9sliderExecute(event):
            J9delta = float(self.J9jogslide.get()) - float(self.J9curAngEntryField.get())
            if J9delta < 0:
                self.J9jogNeg(abs(J9delta))
            else:
                self.J9jogPos(abs(J9delta))

        self.J9jogslide = ctk.CTkSlider(
            self.J9jogFrame, from_=-self.J9NegLim, to=self.J9PosLim, orientation="horizontal", command=J9sliderUpdate, width=125
        )
        self.J9jogslide.bind("<ButtonRelease-1>", J9sliderExecute)
        self.J9jogslide.place(x=10, y=43)

        """ Entry Fields """

        self.progframe = ctk.CTkFrame(self.tab1)
        self.progframe.place(x=7, y=174)

        # Scrollbar and Listbox
        self.scrollbar = ctk.CTkScrollbar(self.progframe)
        self.scrollbar.pack(side="right", fill="y")

        self.progView = ctk.CTkTextbox(
            self.progframe,
            exportselection=0,
            width=680,  # Approximation based on original width (105 chars * ~7px per char)
            height=490,  # Approximation based on original height
            yscrollcommand=self.scrollbar.set,
        )
        self.progView.bind("<<ListboxSelect>>", self.progViewselect)
        self.progView.pack(side="left", fill="both", expand=True)
        self.scrollbar.configure(command=self.progView.yview)

        # Entry Fields
        self.incrementEntryField = ctk.CTkEntry(self.tab1, width=40, justify="center", placeholder_text="Increment")
        self.incrementEntryField.place(x=380, y=45)

        self.curRowEntryField = ctk.CTkEntry(self.tab1, width=40, justify="center", placeholder_text="Current Row")
        self.curRowEntryField.place(x=174, y=120)

        self.manEntryField = ctk.CTkEntry(self.tab1, width=680, placeholder_text="Manual Entry")
        self.manEntryField.place(x=10, y=670)

        self.ProgEntryField = ctk.CTkEntry(self.tab1, width=150, justify="center", placeholder_text="Program")
        self.ProgEntryField.place(x=70, y=45)

        self.speedEntryField = ctk.CTkEntry(self.tab1, width=40, justify="center", placeholder_text="Speed")
        self.speedEntryField.place(x=380, y=80)

        self.ACCspeedField = ctk.CTkEntry(self.tab1, width=40, justify="center", placeholder_text="Accel Speed")
        self.ACCspeedField.place(x=380, y=100)

        self.DECspeedField = ctk.CTkEntry(self.tab1, width=40, justify="center", placeholder_text="Decel Speed")
        self.DECspeedField.place(x=380, y=120)

        self.ACCrampField = ctk.CTkEntry(self.tab1, width=40, justify="center", placeholder_text="Accel Ramp")
        self.ACCrampField.place(x=380, y=140)

        self.roundEntryField = ctk.CTkEntry(self.tab1, width=40, justify="center", placeholder_text="Round")
        self.roundEntryField.place(x=590, y=80)

        # X Entry Field
        self.XcurEntryField = ctk.CTkEntry(self.CartjogFrame, width=40, justify="center")
        self.XcurEntryField.place(x=660, y=195)

        # Y Entry Field
        self.YcurEntryField = ctk.CTkEntry(self.CartjogFrame, width=40, justify="center")
        self.YcurEntryField.place(x=750, y=195)

        # Z Entry Field
        self.ZcurEntryField = ctk.CTkEntry(self.CartjogFrame, width=40, justify="center")
        self.ZcurEntryField.place(x=840, y=195)

        # Rz Entry Field
        self.RzcurEntryField = ctk.CTkEntry(self.CartjogFrame, width=40, justify="center")
        self.RzcurEntryField.place(x=930, y=195)

        # Ry Entry Field
        self.RycurEntryField = ctk.CTkEntry(self.CartjogFrame, width=40, justify="center")
        self.RycurEntryField.place(x=1020, y=195)

        # Rx Entry Field
        self.RxcurEntryField = ctk.CTkEntry(self.CartjogFrame, width=40, justify="center")
        self.RxcurEntryField.place(x=1110, y=195)

        # Function to toggle the visibility of SavePosEntryField
        def posRegFieldVisible(self):
            curCmdtype = self.options.get()
            if curCmdtype in ["Move PR", "OFF PR", "Teach PR"]:
                self.SavePosEntryField.place(x=780, y=183)
            else:
                self.SavePosEntryField.place_forget()

        # Create buttons in a consistent style and place them within the GUI
        self.getSelBut = ctk.CTkButton(self.tab1, text="Get Selected", width=100, height=30, command=self.getSel)
        self.getSelBut.place(x=10, y=700)

        self.manInsBut = ctk.CTkButton(self.tab1, text="Insert", width=100, height=30, command=self.manInsItem)
        self.manInsBut.place(x=115, y=700)

        self.manRepBut = ctk.CTkButton(self.tab1, text="Replace", width=100, height=30, command=self.manReplItem)
        self.manRepBut.place(x=220, y=700)

        self.openTextBut = ctk.CTkButton(self.tab1, text="Open Text", width=100, height=30, command=self.openText)
        self.openTextBut.place(x=325, y=700)

        self.reloadProgBut = ctk.CTkButton(self.tab1, text="Reload", width=100, height=30, command=self.reloadProg)
        self.reloadProgBut.place(x=430, y=700)

        # Dropdown menu for speed options
        self.speedOption = ctk.StringVar(value="Percent")
        self.speedMenu = ctk.CTkOptionMenu(
            self.tab1,
            variable=self.speedOption,
            values=["Percent", "Seconds", "mm per Sec"],
            width=120,
            height=30
        )
        self.speedMenu.place(x=412, y=76)

        """ Single buttons """

        # Dropdown menu for options with callback for visibility
        self.options = ctk.StringVar(value="Move J")
        self.menu = ctk.CTkOptionMenu(
            self.tab1,
            variable=self.options,
            values=[
                "Move J", "OFF J", "Move L", "Move R", "Move A Mid", "Move A End",
                "Move C Center", "Move C Start", "Move C Plane", "Start Spline",
                "End Spline", "Move PR", "OFF PR", "Teach PR", "Move Vis"
            ],
            command=posRegFieldVisible,
            width=150,
            height=30
        )
        self.menu.place(x=700, y=180)

        # Save Position Entry Field
        self.SavePosEntryField = ctk.CTkEntry(self.tab1, width=50, justify="center", placeholder_text="Save Pos")

        # self.SavePosEntryField.place(x=800, y=183)

        # Buttons for various operations
        self.teachInsBut = ctk.CTkButton(
            self.tab1, text="Teach New Position", width=150, height=30, command=self.teachInsertBelSelected
        )
        self.teachInsBut.place(x=700, y=220)

        self.teachReplaceBut = ctk.CTkButton(
            self.tab1, text="Modify Position", width=150, height=30, command=self.teachReplaceSelected
        )
        self.teachReplaceBut.place(x=700, y=260)

        self.deleteBut = ctk.CTkButton(
            self.tab1, text="Delete", width=150, height=30, command=self.deleteitem
        )
        self.deleteBut.place(x=700, y=300)

        self.CalibrateBut = ctk.CTkButton(
            self.tab1, text="Auto Calibrate CMD", width=150, height=30, command=self.insCalibrate
        )
        self.CalibrateBut.place(x=700, y=340)

        self.camOnBut = ctk.CTkButton(
            self.tab1, text="Camera On", width=150, height=30, command=self.cameraOn
        )
        self.camOnBut.place(x=700, y=380)

        self.camOffBut = ctk.CTkButton(
            self.tab1, text="Camera Off", width=150, height=30, command=self.cameraOff
        )
        self.camOffBut.place(x=700, y=420)

        """ Buttons with multiple entry fields """

        # Button and entry for Wait Time
        self.waitTimeBut = ctk.CTkButton(
            self.tab1, text="Wait Time (seconds)", width=150, height=30, command=self.waitTime
        )
        self.waitTimeBut.place(x=700, y=460)

        self.waitTimeEntryField = ctk.CTkEntry(
            self.tab1, width=50, justify="center", placeholder_text="Time"
        )
        self.waitTimeEntryField.place(x=855, y=465)

        # Button and entry for Wait Input ON
        self.waitInputOnBut = ctk.CTkButton(
            self.tab1, text="Wait Input ON", width=150, height=30, command=self.waitInputOn
        )
        self.waitInputOnBut.place(x=700, y=500)

        self.waitInputEntryField = ctk.CTkEntry(
            self.tab1, width=50, justify="center", placeholder_text="Input"
        )
        self.waitInputEntryField.place(x=855, y=505)

        # Button and entry for Wait Input OFF
        self.waitInputOffBut = ctk.CTkButton(
            self.tab1, text="Wait Input OFF", width=150, height=30, command=self.waitInputOff
        )
        self.waitInputOffBut.place(x=700, y=540)

        self.waitInputOffEntryField = ctk.CTkEntry(
            self.tab1, width=50, justify="center", placeholder_text="Input"
        )
        self.waitInputOffEntryField.place(x=855, y=545)

        # Button and entry for Set Output ON
        self.setOutputOnBut = ctk.CTkButton(
            self.tab1, text="Set Output On", width=150, height=30, command=self.setOutputOn
        )
        self.setOutputOnBut.place(x=700, y=580)

        self.outputOnEntryField = ctk.CTkEntry(
            self.tab1, width=50, justify="center", placeholder_text="Output"
        )
        self.outputOnEntryField.place(x=855, y=585)

        # Button and entry for Set Output OFF
        self.setOutputOffBut = ctk.CTkButton(
            self.tab1, text="Set Output OFF", width=150, height=30, command=self.setOutputOff
        )
        self.setOutputOffBut.place(x=700, y=620)

        self.outputOffEntryField = ctk.CTkEntry(
            self.tab1, width=50, justify="center", placeholder_text="Output"
        )
        self.outputOffEntryField.place(x=855, y=625)

        # Button and entry for Create Tab
        self.tabNumBut = ctk.CTkButton(
            self.tab1, text="Create Tab", width=150, height=30, command=self.tabNumber
        )
        self.tabNumBut.place(x=700, y=660)

        self.tabNumEntryField = ctk.CTkEntry(
            self.tab1, width=50, justify="center", placeholder_text="Tab"
        )
        self.tabNumEntryField.place(x=855, y=665)

        # Button and entry for Jump to Tab
        self.jumpTabBut = ctk.CTkButton(
            self.tab1, text="Jump to Tab", width=150, height=30, command=self.jumpTab
        )
        self.jumpTabBut.place(x=700, y=700)

        self.jumpTabEntryField = ctk.CTkEntry(
            self.tab1, width=50, justify="center", placeholder_text="Tab"
        )
        self.jumpTabEntryField.place(x=855, y=705)

        """ Buttons with multiple entry fields """

        # IF Command Section
        self.ifSelLabel = ctk.CTkLabel(
            self.tab1, text="IF", font=("Arial", 10, "bold")
        )
        self.ifSelLabel.place(x=950, y=363)

        self.iFOption = ctk.StringVar()
        self.ifMenu = ctk.CTkOptionMenu(
            self.tab1,
            variable=self.iFOption,
            values=["Input", "Register", "COM Device"],
            width=80,
        )
        self.ifMenu.place(x=970, y=360)

        self.ifVarEntry = ctk.CTkEntry(
            self.tab1, width=50, justify="center", placeholder_text="Var"
        )
        self.ifVarEntry.place(x=1065, y=363)

        self.ifEqualLabel = ctk.CTkLabel(
            self.tab1, text="=", font=("Arial", 10, "bold")
        )
        self.ifEqualLabel.place(x=1102, y=363)

        self.ifInputEntry = ctk.CTkEntry(
            self.tab1, width=50, justify="center", placeholder_text="Input"
        )
        self.ifInputEntry.place(x=1115, y=363)

        self.iFSelection = ctk.StringVar()
        self.ifSelMenu = ctk.CTkOptionMenu(
            self.tab1,
            variable=self.iFSelection,
            values=["Call Prog", "Jump Tab"],
            width=90,
        )
        self.ifSelMenu.place(x=1160, y=360)

        self.ifDestEntry = ctk.CTkEntry(
            self.tab1, width=90, justify="center", placeholder_text="Dest"
        )
        self.ifDestEntry.place(x=1260, y=363)

        self.ifDotLabel = ctk.CTkLabel(
            self.tab1, text="•", font=("Arial", 10, "bold")
        )
        self.ifDotLabel.place(x=1325, y=363)

        self.insertIfCMDBut = ctk.CTkButton(
            self.tab1, text="Insert IF CMD", width=100, height=30, command=self.IfCMDInsert
        )
        self.insertIfCMDBut.place(x=1340, y=359)

        # Buttons for G-Code and Other Actions
        self.gcPlayBut = ctk.CTkButton(
            self.tab1, text="Play Gcode", width=150, height=30, command=self.insertGCprog
        )
        self.gcPlayBut.place(x=950, y=400)

        self.readAuxComBut = ctk.CTkButton(
            self.tab1, text="Read COM Device", width=150, height=30, command=self.ReadAuxCom
        )
        self.readAuxComBut.place(x=950, y=440)

        self.servoBut = ctk.CTkButton(
            self.tab1, text="Servo", width=150, height=30, command=self.Servo
        )
        self.servoBut.place(x=950, y=480)

        self.regNumBut = ctk.CTkButton(
            self.tab1, text="Register", width=150, height=30, command=self.insertRegister
        )
        self.regNumBut.place(x=950, y=520)

        self.storPosBut = ctk.CTkButton(
            self.tab1, text="Position Register", width=150, height=30, command=self.storPos
        )
        self.storPosBut.place(x=950, y=560)

        self.callBut = ctk.CTkButton(
            self.tab1, text="Call Program", width=150, height=30, command=self.insertCallProg
        )
        self.callBut.place(x=950, y=600)

        self.returnBut = ctk.CTkButton(
            self.tab1, text="Return", width=150, height=30, command=self.insertReturn
        )
        self.returnBut.place(x=950, y=640)

        self.visFindBut = ctk.CTkButton(
            self.tab1, text="Vision Find", width=150, height=30, command=self.insertvisFind
        )
        self.visFindBut.place(x=950, y=680)

        # Entry Fields
        self.PlayGCEntryField = ctk.CTkEntry(self.tab1, width=150, justify="center")
        self.PlayGCEntryField.place(x=1107, y=403)

        self.auxPortEntryField = ctk.CTkEntry(self.tab1, width=50, justify="center")
        self.auxPortEntryField.place(x=1107, y=443)

        self.auxCharEntryField = ctk.CTkEntry(self.tab1, width=50, justify="center")
        self.auxCharEntryField.place(x=1147, y=443)

        self.servoNumEntryField = ctk.CTkEntry(self.tab1, width=50, justify="center")
        self.servoNumEntryField.place(x=1107, y=483)

        self.servoPosEntryField = ctk.CTkEntry(self.tab1, width=50, justify="center")
        self.servoPosEntryField.place(x=1147, y=483)

        self.regNumEntryField = ctk.CTkEntry(self.tab1, width=50, justify="center")
        self.regNumEntryField.place(x=1107, y=523)

        self.regEqEntryField = ctk.CTkEntry(self.tab1, width=50, justify="center")
        self.regEqEntryField.place(x=1147, y=523)

        self.storPosNumEntryField = ctk.CTkEntry(self.tab1, width=50, justify="center")
        self.storPosNumEntryField.place(x=1107, y=563)

        self.storPosElEntryField = ctk.CTkEntry(self.tab1, width=50, justify="center")
        self.storPosElEntryField.place(x=1147, y=563)

        self.storPosValEntryField = ctk.CTkEntry(self.tab1, width=50, justify="center")
        self.storPosValEntryField.place(x=1187, y=563)

        self.changeProgEntryField = ctk.CTkEntry(self.tab1, width=150, justify="center")
        self.changeProgEntryField.place(x=1107, y=603)

        self.visPassEntryField = ctk.CTkEntry(self.tab1, width=50, justify="center")
        self.visPassEntryField.place(x=1107, y=683)

        self.visFailEntryField = ctk.CTkEntry(self.tab1, width=50, justify="center")
        self.visFailEntryField.place(x=1147, y=683)

        # Labels
        self.manEntLab = ctk.CTkLabel(self.tab1, font=("Arial", 6), text="Manual Program Entry")
        self.manEntLab.place(x=10, y=685)

        self.auxComLab = ctk.CTkLabel(self.tab1, font=("Arial", 6), text="Port             Char")
        self.auxComLab.place(x=1107, y=429)

        self.servoLab = ctk.CTkLabel(self.tab1, font=("Arial", 6), text="Number      Position")
        self.servoLab.place(x=1107, y=469)

        self.regEqLab = ctk.CTkLabel(self.tab1, font=("Arial", 6), text="Register       (++/--)")
        self.regEqLab.place(x=1107, y=509)

        self.storPosEqLab = ctk.CTkLabel(
            self.tab1, font=("Arial", 6), text=" Pos Reg      Element       (++/--)"
        )
        self.storPosEqLab.place(x=1107, y=549)

        self.visPassLab = ctk.CTkLabel(self.tab1, font=("Arial", 6), text="Pass Tab     Fail Tab")
        self.visPassLab.place(x=1107, y=670)

        # Load and Create Program Buttons
        self.loadProgBut = ctk.CTkButton(
            self.tab1, text="Load", width=80, height=30, command=self.loadProg
        )
        self.loadProgBut.place(x=202, y=42)

        self.createProgBut = ctk.CTkButton(
            self.tab1, text="New Prog", width=80, height=30, command=self.CreateProg
        )
        self.createProgBut.place(x=285, y=42)

        # Play, Stop, and Other Icons
        self.runProgBut = ctk.CTkButton(
            self.tab1, 
            text="Play",
            width=80, 
            image=ctk.CTkImage(Image.open(os.path.join('assets', 'play-icon.png'))), 
            command=self.runProg
        )
        self.runProgBut.place(x=20, y=80)

        self.xboxBut = ctk.CTkButton(
            self.tab1, 
            text="Controller", 
            width=80,
            image=ctk.CTkImage(Image.open(os.path.join('assets', 'xbox-icon.png'))), 
            command=self.xbox
        )
        self.xboxBut.place(x=700, y=80)

        self.stopProgBut = ctk.CTkButton(
            self.tab1, 
            text="Stop", 
            width=80,
            image=ctk.CTkImage(Image.open(os.path.join('assets', 'stop-icon.png'))), 
            command=self.stopProg
        )
        self.stopProgBut.place(x=220, y=80)

        self.revBut = ctk.CTkButton(
            self.tab1, text="REV", width=50, command=self.stepRev
        )
        self.revBut.place(x=105, y=80)

        self.fwdBut = ctk.CTkButton(
            self.tab1, text="FWD", width=50, command=self.stepFwd
        )
        self.fwdBut.place(x=160, y=80)

        # Checkbutton
        self.IncJogCbut = ctk.CTkCheckBox(
            self.tab1, text="Incremental Jog", variable=self.IncJogStat
        )
        self.IncJogCbut.place(x=412, y=46)

        """ XYZ Jog Buttons """

        def SelXjogNeg(self, event=None):
            IncJogStatVal = int(self.IncJogStat.get())
            if IncJogStatVal == 1:
                self.XjogNeg(float(self.incrementEntryField.get()))
            else:
                self.LiveCarJog(10)

        def SelXjogPos(self, event=None):
            IncJogStatVal = int(self.IncJogStat.get())
            if IncJogStatVal == 1:
                self.XjogPos(float(self.incrementEntryField.get()))
            else:
                self.LiveCarJog(11)

        def SelYjogNeg(self, event=None):
            IncJogStatVal = int(self.IncJogStat.get())
            if IncJogStatVal == 1:
                self.YjogNeg(float(self.incrementEntryField.get()))
            else:
                self.LiveCarJog(20)

        def SelYjogPos(self, event=None):
            IncJogStatVal = int(self.IncJogStat.get())
            if IncJogStatVal == 1:
                self.YjogPos(float(self.incrementEntryField.get()))
            else:
                self.LiveCarJog(21)

        def SelZjogNeg(self, event=None):
            IncJogStatVal = int(self.IncJogStat.get())
            if IncJogStatVal == 1:
                self.ZjogNeg(float(self.incrementEntryField.get()))
            else:
                self.LiveCarJog(30)

        def SelZjogPos(self, event=None):
            IncJogStatVal = int(self.IncJogStat.get())
            if IncJogStatVal == 1:
                self.ZjogPos(float(self.incrementEntryField.get()))
            else:
                self.LiveCarJog(31)

        # Buttons for Cart Jogging
        self.XjogNegBut = ctk.CTkButton(self.CartjogFrame, text="-", width=30, height=25)
        self.XjogNegBut.bind("<ButtonPress>", SelXjogNeg)
        self.XjogNegBut.bind("<ButtonRelease>", self.StopJog)
        self.XjogNegBut.place(x=642, y=225)

        self.XjogPosBut = ctk.CTkButton(self.CartjogFrame, text="+", width=30, height=25)
        self.XjogPosBut.bind("<ButtonPress>", SelXjogPos)
        self.XjogPosBut.bind("<ButtonRelease>", self.StopJog)
        self.XjogPosBut.place(x=680, y=225)

        self.YjogNegBut = ctk.CTkButton(self.CartjogFrame, text="-", width=30, height=25)
        self.YjogNegBut.bind("<ButtonPress>", SelYjogNeg)
        self.YjogNegBut.bind("<ButtonRelease>", self.StopJog)
        self.YjogNegBut.place(x=732, y=225)

        self.YjogPosBut = ctk.CTkButton(self.CartjogFrame, text="+", width=30, height=25)
        self.YjogPosBut.bind("<ButtonPress>", SelYjogPos)
        self.YjogPosBut.bind("<ButtonRelease>", self.StopJog)
        self.YjogPosBut.place(x=770, y=225)

        self.ZjogNegBut = ctk.CTkButton(self.CartjogFrame, text="-", width=30, height=25)
        self.ZjogNegBut.bind("<ButtonPress>", SelZjogNeg)
        self.ZjogNegBut.bind("<ButtonRelease>", self.StopJog)
        self.ZjogNegBut.place(x=822, y=225)

        self.ZjogPosBut = ctk.CTkButton(self.CartjogFrame, text="+", width=30, height=25)
        self.ZjogPosBut.bind("<ButtonPress>", SelZjogPos)
        self.ZjogPosBut.bind("<ButtonRelease>", self.StopJog)
        self.ZjogPosBut.place(x=860, y=225)

        """ R(XYZ) Jog Buttons """

        def SelRzjogNeg(self, event=None):
            IncJogStatVal = int(self.IncJogStat.get())
            if IncJogStatVal == 1:
                self.RzjogNeg(float(self.incrementEntryField.get()))
            else:
                self.LiveCarJog(40)

        def SelRzjogPos(self, event=None):
            IncJogStatVal = int(self.IncJogStat.get())
            if IncJogStatVal == 1:
                self.RzjogPos(float(self.incrementEntryField.get()))
            else:
                self.LiveCarJog(41)

        def SelRyjogNeg(self, event=None):
            IncJogStatVal = int(self.IncJogStat.get())
            if IncJogStatVal == 1:
                self.RyjogNeg(float(self.incrementEntryField.get()))
            else:
                self.LiveCarJog(50)

        def SelRyjogPos(self, event=None):
            IncJogStatVal = int(self.IncJogStat.get())
            if IncJogStatVal == 1:
                self.RyjogPos(float(self.incrementEntryField.get()))
            else:
                self.LiveCarJog(51)

        def SelRxjogNeg(self, event=None):
            IncJogStatVal = int(self.IncJogStat.get())
            if IncJogStatVal == 1:
                self.RxjogNeg(float(self.incrementEntryField.get()))
            else:
                self.LiveCarJog(60)

        def SelRxjogPos(self, event=None):
            IncJogStatVal = int(self.IncJogStat.get())
            if IncJogStatVal == 1:
                self.RxjogPos(float(self.incrementEntryField.get()))
            else:
                self.LiveCarJog(61)

        # Buttons for Rotation Jogging
        self.RzjogNegBut = ctk.CTkButton(self.CartjogFrame, text="-", width=30, height=25)
        self.RzjogNegBut.bind("<ButtonPress>", SelRzjogNeg)
        self.RzjogNegBut.bind("<ButtonRelease>", self.StopJog)
        self.RzjogNegBut.place(x=912, y=225)

        self.RzjogPosBut = ctk.CTkButton(self.CartjogFrame, text="+", width=30, height=25)
        self.RzjogPosBut.bind("<ButtonPress>", SelRzjogPos)
        self.RzjogPosBut.bind("<ButtonRelease>", self.StopJog)
        self.RzjogPosBut.place(x=950, y=225)

        self.RyjogNegBut = ctk.CTkButton(self.CartjogFrame, text="-", width=30, height=25)
        self.RyjogNegBut.bind("<ButtonPress>", SelRyjogNeg)
        self.RyjogNegBut.bind("<ButtonRelease>", self.StopJog)
        self.RyjogNegBut.place(x=1002, y=225)

        self.RyjogPosBut = ctk.CTkButton(self.CartjogFrame, text="+", width=30, height=25)
        self.RyjogPosBut.bind("<ButtonPress>", SelRyjogPos)
        self.RyjogPosBut.bind("<ButtonRelease>", self.StopJog)
        self.RyjogPosBut.place(x=1040, y=225)

        self.RxjogNegBut = ctk.CTkButton(self.CartjogFrame, text="-", width=30, height=25)
        self.RxjogNegBut.bind("<ButtonPress>", SelRxjogNeg)
        self.RxjogNegBut.bind("<ButtonRelease>", self.StopJog)
        self.RxjogNegBut.place(x=1092, y=225)

        self.RxjogPosBut = ctk.CTkButton(self.CartjogFrame, text="+", width=30, height=25)
        self.RxjogPosBut.bind("<ButtonPress>", SelRxjogPos)
        self.RxjogPosBut.bind("<ButtonRelease>", self.StopJog)
        self.RxjogPosBut.place(x=1130, y=225)

        """ T(XYZ) Jog Buttons """

        def SelTxjogNeg(self, event=None):
            IncJogStatVal = int(self.IncJogStat.get())
            if IncJogStatVal == 1:
                self.TXjogNeg(float(self.incrementEntryField.get()))
            else:
                self.LiveToolJog(10)

        def SelTxjogPos(self, event=None):
            IncJogStatVal = int(self.IncJogStat.get())
            if IncJogStatVal == 1:
                self.TXjogPos(float(self.incrementEntryField.get()))
            else:
                self.LiveToolJog(11)

        def SelTyjogNeg(self, event=None):
            IncJogStatVal = int(self.IncJogStat.get())
            if IncJogStatVal == 1:
                self.TYjogNeg(float(self.incrementEntryField.get()))
            else:
                self.LiveToolJog(20)

        def SelTyjogPos(self, event=None):
            IncJogStatVal = int(self.IncJogStat.get())
            if IncJogStatVal == 1:
                self.TYjogPos(float(self.incrementEntryField.get()))
            else:
                self.LiveToolJog(21)

        def SelTzjogNeg(self, event=None):
            IncJogStatVal = int(self.IncJogStat.get())
            if IncJogStatVal == 1:
                self.TZjogNeg(float(self.incrementEntryField.get()))
            else:
                self.LiveToolJog(30)

        def SelTzjogPos(self, event=None):
            IncJogStatVal = int(self.IncJogStat.get())
            if IncJogStatVal == 1:
                self.TZjogPos(float(self.incrementEntryField.get()))
            else:
                self.LiveToolJog(31)

        # Buttons for Tool Jogging
        self.TXjogNegBut = ctk.CTkButton(self.CartjogFrame, text="-", width=30, height=25)
        self.TXjogNegBut.bind("<ButtonPress>", SelTxjogNeg)
        self.TXjogNegBut.bind("<ButtonRelease>", self.StopJog)
        self.TXjogNegBut.place(x=642, y=300)

        self.TXjogPosBut = ctk.CTkButton(self.CartjogFrame, text="+", width=30, height=25)
        self.TXjogPosBut.bind("<ButtonPress>", SelTxjogPos)
        self.TXjogPosBut.bind("<ButtonRelease>", self.StopJog)
        self.TXjogPosBut.place(x=680, y=300)

        self.TYjogNegBut = ctk.CTkButton(self.CartjogFrame, text="-", width=30, height=25)
        self.TYjogNegBut.bind("<ButtonPress>", SelTyjogNeg)
        self.TYjogNegBut.bind("<ButtonRelease>", self.StopJog)
        self.TYjogNegBut.place(x=732, y=300)

        self.TYjogPosBut = ctk.CTkButton(self.CartjogFrame, text="+", width=30, height=25)
        self.TYjogPosBut.bind("<ButtonPress>", SelTyjogPos)
        self.TYjogPosBut.bind("<ButtonRelease>", self.StopJog)
        self.TYjogPosBut.place(x=770, y=300)

        self.TZjogNegBut = ctk.CTkButton(self.CartjogFrame, text="-", width=30, height=25)
        self.TZjogNegBut.bind("<ButtonPress>", SelTzjogNeg)
        self.TZjogNegBut.bind("<ButtonRelease>", self.StopJog)
        self.TZjogNegBut.place(x=822, y=300)

        self.TZjogPosBut = ctk.CTkButton(self.CartjogFrame, text="+", width=30, height=25)
        self.TZjogPosBut.bind("<ButtonPress>", SelTzjogPos)
        self.TZjogPosBut.bind("<ButtonRelease>", self.StopJog)
        self.TZjogPosBut.place(x=860, y=300)

        """ T(R(XYZ)) Jog Buttons """

        def SelTRzjogNeg(self, event=None):
            IncJogStatVal = int(self.IncJogStat.get())
            if IncJogStatVal == 1:
                self.TRzjogNeg(float(self.incrementEntryField.get()))
            else:
                self.LiveToolJog(40)

        def SelTRzjogPos(self, event=None):
            IncJogStatVal = int(self.IncJogStat.get())
            if IncJogStatVal == 1:
                self.TRzjogPos(float(self.incrementEntryField.get()))
            else:
                self.LiveToolJog(41)

        def SelTRyjogNeg(self, event=None):
            IncJogStatVal = int(self.IncJogStat.get())
            if IncJogStatVal == 1:
                self.TRyjogNeg(float(self.incrementEntryField.get()))
            else:
                self.LiveToolJog(50)

        def SelTRyjogPos(self, event=None):
            IncJogStatVal = int(self.IncJogStat.get())
            if IncJogStatVal == 1:
                self.TRyjogPos(float(self.incrementEntryField.get()))
            else:
                self.LiveToolJog(51)

        def SelTRxjogNeg(self, event=None):
            IncJogStatVal = int(self.IncJogStat.get())
            if IncJogStatVal == 1:
                self.TRxjogNeg(float(self.incrementEntryField.get()))
            else:
                self.LiveToolJog(60)

        def SelTRxjogPos(self, event=None):
            IncJogStatVal = int(self.IncJogStat.get())
            if IncJogStatVal == 1:
                self.TRxjogPos(float(self.incrementEntryField.get()))
            else:
                self.LiveToolJog(61)

        # Buttons for Tool Rotation Jogging
        self.TRzjogNegBut = ctk.CTkButton(self.CartjogFrame, text="-", width=30, height=25)
        self.TRzjogNegBut.bind("<ButtonPress>", SelTRzjogNeg)
        self.TRzjogNegBut.bind("<ButtonRelease>", self.StopJog)
        self.TRzjogNegBut.place(x=912, y=300)

        self.TRzjogPosBut = ctk.CTkButton(self.CartjogFrame, text="+", width=30, height=25)
        self.TRzjogPosBut.bind("<ButtonPress>", SelTRzjogPos)
        self.TRzjogPosBut.bind("<ButtonRelease>", self.StopJog)
        self.TRzjogPosBut.place(x=950, y=300)

        self.TRyjogNegBut = ctk.CTkButton(self.CartjogFrame, text="-", width=30, height=25)
        self.TRyjogNegBut.bind("<ButtonPress>", SelTRyjogNeg)
        self.TRyjogNegBut.bind("<ButtonRelease>", self.StopJog)
        self.TRyjogNegBut.place(x=1002, y=300)

        self.TRyjogPosBut = ctk.CTkButton(self.CartjogFrame, text="+", width=30, height=25)
        self.TRyjogPosBut.bind("<ButtonPress>", SelTRyjogPos)
        self.TRyjogPosBut.bind("<ButtonRelease>", self.StopJog)
        self.TRyjogPosBut.place(x=1040, y=300)

        self.TRxjogNegBut = ctk.CTkButton(self.CartjogFrame, text="-", width=30, height=25)
        self.TRxjogNegBut.bind("<ButtonPress>", SelTRxjogNeg)
        self.TRxjogNegBut.bind("<ButtonRelease>", self.StopJog)
        self.TRxjogNegBut.place(x=1092, y=300)

        self.TRxjogPosBut = ctk.CTkButton(self.CartjogFrame, text="+", width=30, height=25)
        self.TRxjogPosBut.bind("<ButtonPress>", SelTRxjogPos)
        self.TRxjogPosBut.bind("<ButtonRelease>", self.StopJog)
        self.TRxjogPosBut.place(x=1130, y=300)

        ## TAB 2 LABELS ##

        self.ComPortLab = ctk.CTkLabel(self.tab2, text="TEENSY COM PORT:")
        self.ComPortLab.place(x=66, y=90)

        self.ComPortLab2 = ctk.CTkLabel(self.tab2, text="IO BOARD COM PORT:")
        self.ComPortLab2.place(x=60, y=160)

        self.ComPortLab3 = ctk.CTkLabel(self.tab2, text="TEST AUX COM DEVICE")
        self.ComPortLab3.place(x=60, y=230)

        self.AuxPortNumLab = ctk.CTkLabel(self.tab2, text="Aux Com Port")
        self.AuxPortNumLab.place(x=84, y=254)

        self.AuxPortCharLab = ctk.CTkLabel(self.tab2, text="Char to Read")
        self.AuxPortCharLab.place(x=84, y=284)

        self.almStatusLab2 = ctk.CTkLabel(self.tab2, text="SYSTEM READY - NO ACTIVE ALARMS")
        self.almStatusLab2.place(x=25, y=20)

        self.comLab = ctk.CTkLabel(self.tab2, text="Communication")
        self.comLab.place(x=72, y=60)

        # Calibration Labels
        self.jointCalLab = ctk.CTkLabel(self.tab2, text="Robot Calibration")
        self.jointCalLab.place(x=290, y=60)

        # 7th Axis Labels
        self.axis7Lab = ctk.CTkLabel(self.tab2, text="7th Axis Calibration")
        self.axis7Lab.place(x=665, y=300)

        self.axis7lengthLab = ctk.CTkLabel(self.tab2, text="7th Axis Length:")
        self.axis7lengthLab.place(x=651, y=340)

        self.axis7rotLab = ctk.CTkLabel(self.tab2, text="MM per Rotation:")
        self.axis7rotLab.place(x=645, y=370)

        self.axis7stepsLab = ctk.CTkLabel(self.tab2, text="Drive Steps:")
        self.axis7stepsLab.place(x=675, y=400)

        self.axis7pinsetLab = ctk.CTkLabel(
            self.tab2, font=("Arial", 8), text="StepPin = 12 / DirPin = 13 / CalPin = 36"
        )
        self.axis7pinsetLab.place(x=627, y=510)

        # 8th Axis Labels
        self.axis8Lab = ctk.CTkLabel(self.tab2, text="8th Axis Calibration")
        self.axis8Lab.place(x=865, y=300)

        self.axis8lengthLab = ctk.CTkLabel(self.tab2, text="8th Axis Length:")
        self.axis8lengthLab.place(x=851, y=340)

        self.axis8rotLab = ctk.CTkLabel(self.tab2, text="MM per Rotation:")
        self.axis8rotLab.place(x=845, y=370)

        self.axis8stepsLab = ctk.CTkLabel(self.tab2, text="Drive Steps:")
        self.axis8stepsLab.place(x=875, y=400)

        self.axis8pinsetLab = ctk.CTkLabel(
            self.tab2, font=("Arial", 8), text="StepPin = 32 / DirPin = 33 / CalPin = 37"
        )
        self.axis8pinsetLab.place(x=827, y=510)

        # 9th Axis Labels
        self.axis9Lab = ctk.CTkLabel(self.tab2, text="9th Axis Calibration")
        self.axis9Lab.place(x=1065, y=300)

        self.axis9lengthLab = ctk.CTkLabel(self.tab2, text="9th Axis Length:")
        self.axis9lengthLab.place(x=1051, y=340)

        self.axis9rotLab = ctk.CTkLabel(self.tab2, text="MM per Rotation:")
        self.axis9rotLab.place(x=1045, y=370)

        self.axis9stepsLab = ctk.CTkLabel(self.tab2, text="Drive Steps:")
        self.axis9stepsLab.place(x=1075, y=400)

        self.axis9pinsetLab = ctk.CTkLabel(
            self.tab2, font=("Arial", 8), text="StepPin = 34 / DirPin = 35 / CalPin = 38"
        )
        self.axis9pinsetLab.place(x=1027, y=510)

        # Calibration Offsets
        self.CalibrationOffsetsLab = ctk.CTkLabel(self.tab2, text="Calibration Offsets")
        self.CalibrationOffsetsLab.place(x=485, y=60)

        self.J1calLab = ctk.CTkLabel(self.tab2, text="J1 Offset")
        self.J1calLab.place(x=480, y=90)

        self.J2calLab = ctk.CTkLabel(self.tab2, text="J2 Offset")
        self.J2calLab.place(x=480, y=120)

        self.J3calLab = ctk.CTkLabel(self.tab2, text="J3 Offset")
        self.J3calLab.place(x=480, y=150)

        self.J4calLab = ctk.CTkLabel(self.tab2, text="J4 Offset")
        self.J4calLab.place(x=480, y=180)

        self.J5calLab = ctk.CTkLabel(self.tab2, text="J5 Offset")
        self.J5calLab.place(x=480, y=210)

        self.J6calLab = ctk.CTkLabel(self.tab2, text="J6 Offset")
        self.J6calLab.place(x=480, y=240)

        self.J7calLab = ctk.CTkLabel(self.tab2, text="J7 Offset")
        self.J7calLab.place(x=480, y=280)

        self.J8calLab = ctk.CTkLabel(self.tab2, text="J8 Offset")
        self.J8calLab.place(x=480, y=310)

        self.J9calLab = ctk.CTkLabel(self.tab2, text="J9 Offset")
        self.J9calLab.place(x=480, y=340)

        # Encoder Control
        self.CalibrationOffsetsLab2 = ctk.CTkLabel(self.tab2, text="Encoder Control")
        self.CalibrationOffsetsLab2.place(x=715, y=60)

        # Command Status
        self.cmdSentLab = ctk.CTkLabel(self.tab2, text="Last Command Sent to Controller")
        self.cmdSentLab.place(x=10, y=565)

        self.cmdRecLab = ctk.CTkLabel(self.tab2, text="Last Response From Controller")
        self.cmdRecLab.place(x=10, y=625)

        # Theme
        self.ThemeLab = ctk.CTkLabel(self.tab2, text="Theme")
        self.ThemeLab.place(x=925, y=60)

        """ Tab 2 Buttons """

        self.comPortBut = ctk.CTkButton(self.tab2, text="Set Com Teensy", command=self.setCom, width=120)
        self.comPortBut.place(x=85, y=110)

        self.comPortBut2 = ctk.CTkButton(self.tab2, text="Set Com IO Board", command=self.setCom2, width=120)
        self.comPortBut2.place(x=85, y=180)

        self.comPortBut3 = ctk.CTkButton(self.tab2, text="Test Aux COM Device", command=self.TestAuxCom, width=140)
        self.comPortBut3.place(x=50, y=315)

        self.lightBut = ctk.CTkButton(self.tab2, text="Light", command=self.lightTheme, width=60)
        self.lightBut.place(x=890, y=90)

        self.darkBut = ctk.CTkButton(self.tab2, text="Dark", command=self.darkTheme, width=60)
        self.darkBut.place(x=950, y=90)

        self.autoCalBut = ctk.CTkButton(self.tab2, text="Auto Calibrate", command=self.calRobotAll, width=120)
        self.autoCalBut.place(x=285, y=90)

        # Calibration Checkboxes
        self.J1calCbut = ctk.CTkCheckBox(self.tab2, text="J1", variable=self.J1CalStat, width=40)
        self.J1calCbut.place(x=285, y=125)

        self.J2calCbut = ctk.CTkCheckBox(self.tab2, text="J2", variable=self.J2CalStat, width=40)
        self.J2calCbut.place(x=320, y=125)

        self.J3calCbut = ctk.CTkCheckBox(self.tab2, text="J3", variable=self.J3CalStat, width=40)
        self.J3calCbut.place(x=355, y=125)

        self.J4calCbut = ctk.CTkCheckBox(self.tab2, text="J4", variable=self.J4CalStat, width=40)
        self.J4calCbut.place(x=285, y=145)

        self.J5calCbut = ctk.CTkCheckBox(self.tab2, text="J5", variable=self.J5CalStat, width=40)
        self.J5calCbut.place(x=320, y=145)

        self.J6calCbut = ctk.CTkCheckBox(self.tab2, text="J6", variable=self.J6CalStat, width=40)
        self.J6calCbut.place(x=355, y=145)

        self.J7calCbut = ctk.CTkCheckBox(self.tab2, text="J7", variable=self.J7CalStat, width=40)
        self.J7calCbut.place(x=285, y=165)

        self.J8calCbut = ctk.CTkCheckBox(self.tab2, text="J8", variable=self.J8CalStat, width=40)
        self.J8calCbut.place(x=320, y=165)

        self.J9calCbut = ctk.CTkCheckBox(self.tab2, text="J9", variable=self.J9CalStat, width=40)
        self.J9calCbut.place(x=355, y=165)

        self.J1calCbut2 = ctk.CTkCheckBox(self.tab2, text="J1", variable=self.J1CalStat2, width=40)
        self.J1calCbut2.place(x=285, y=200)

        self.J2calCbut2 = ctk.CTkCheckBox(self.tab2, text="J2", variable=self.J2CalStat2, width=40)
        self.J2calCbut2.place(x=320, y=200)

        self.J3calCbut2 = ctk.CTkCheckBox(self.tab2, text="J3", variable=self.J3CalStat2, width=40)
        self.J3calCbut2.place(x=355, y=200)

        self.J4calCbut2 = ctk.CTkCheckBox(self.tab2, text="J4", variable=self.J4CalStat2, width=40)
        self.J4calCbut2.place(x=285, y=220)

        self.J5calCbut2 = ctk.CTkCheckBox(self.tab2, text="J5", variable=self.J5CalStat2, width=40)
        self.J5calCbut2.place(x=320, y=220)

        self.J6calCbut2 = ctk.CTkCheckBox(self.tab2, text="J6", variable=self.J6CalStat2, width=40)
        self.J6calCbut2.place(x=355, y=220)

        self.J7calCbut2 = ctk.CTkCheckBox(self.tab2, text="J7", variable=self.J7CalStat2, width=40)
        self.J7calCbut2.place(x=285, y=240)

        self.J8calCbut2 = ctk.CTkCheckBox(self.tab2, text="J8", variable=self.J8CalStat2, width=40)
        self.J8calCbut2.place(x=320, y=240)

        self.J9calCbut2 = ctk.CTkCheckBox(self.tab2, text="J9", variable=self.J9CalStat2, width=40)
        self.J9calCbut2.place(x=355, y=240)

        # Axis Calibration Buttons
        self.J7zerobut = ctk.CTkButton(self.tab2, text="Set Axis 7 Calibration to Zero", command=self.zeroAxis7, width=200)
        self.J7zerobut.place(x=627, y=440)

        self.J8zerobut = ctk.CTkButton(self.tab2, text="Set Axis 8 Calibration to Zero", command=self.zeroAxis8, width=200)
        self.J8zerobut.place(x=827, y=440)

        self.J9zerobut = ctk.CTkButton(self.tab2, text="Set Axis 9 Calibration to Zero", command=self.zeroAxis9, width=200)
        self.J9zerobut.place(x=1027, y=440)

        self.J7calbut = ctk.CTkButton(self.tab2, text="Autocalibrate Axis 7", command=self.calRobotJ7, width=200)
        self.J7calbut.place(x=627, y=475)

        self.J8calbut = ctk.CTkButton(self.tab2, text="Autocalibrate Axis 8", command=self.calRobotJ8, width=200)
        self.J8calbut.place(x=827, y=475)

        self.J9calbut = ctk.CTkButton(self.tab2, text="Autocalibrate Axis 9", command=self.calRobotJ9, width=200)
        self.J9calbut.place(x=1027, y=475)

        # Individual Joint Calibration Buttons
        self.CalJ1But = ctk.CTkButton(self.tab2, text="Calibrate J1 Only", command=self.calRobotJ1, width=120)
        self.CalJ1But.place(x=285, y=275)

        self.CalJ2But = ctk.CTkButton(self.tab2, text="Calibrate J2 Only", command=self.calRobotJ2, width=120)
        self.CalJ2But.place(x=285, y=310)

        self.CalJ3But = ctk.CTkButton(self.tab2, text="Calibrate J3 Only", command=self.calRobotJ3, width=120)
        self.CalJ3But.place(x=285, y=345)

        self.CalJ4But = ctk.CTkButton(self.tab2, text="Calibrate J4 Only", command=self.calRobotJ4, width=120)
        self.CalJ4But.place(x=285, y=380)

        self.CalJ5But = ctk.CTkButton(self.tab2, text="Calibrate J5 Only", command=self.calRobotJ5, width=120)
        self.CalJ5But.place(x=285, y=415)

        self.CalJ6But = ctk.CTkButton(self.tab2, text="Calibrate J6 Only", command=self.calRobotJ6, width=120)
        self.CalJ6But.place(x=285, y=450)

        self.CalZeroBut = ctk.CTkButton(self.tab2, text="Force CaL to Home", command=self.CalZeroPos, width=140)
        self.CalZeroBut.place(x=270, y=485)

        self.CalRestBut = ctk.CTkButton(self.tab2, text="Force Cal to Rest", command=self.CalRestPos, width=140)
        self.CalRestBut.place(x=270, y=520)

        # Open Loop Checkboxes
        self.J1OpenLoopCbut = ctk.CTkCheckBox(self.tab2, text="J1 Open Loop (disable encoder)", variable=self.J1OpenLoopStat, width=200)
        self.J1OpenLoopCbut.place(x=665, y=90)

        self.J2OpenLoopCbut = ctk.CTkCheckBox(self.tab2, text="J2 Open Loop (disable encoder)", variable=self.J2OpenLoopStat, width=200)
        self.J2OpenLoopCbut.place(x=665, y=110)

        self.J3OpenLoopCbut = ctk.CTkCheckBox(self.tab2, text="J3 Open Loop (disable encoder)", variable=self.J3OpenLoopStat, width=200)
        self.J3OpenLoopCbut.place(x=665, y=130)

        self.J4OpenLoopCbut = ctk.CTkCheckBox(self.tab2, text="J4 Open Loop (disable encoder)", variable=self.J4OpenLoopStat, width=200)
        self.J4OpenLoopCbut.place(x=665, y=150)

        self.J5OpenLoopCbut = ctk.CTkCheckBox(self.tab2, text="J5 Open Loop (disable encoder)", variable=self.J5OpenLoopStat, width=200)
        self.J5OpenLoopCbut.place(x=665, y=170)

        self.J6OpenLoopCbut = ctk.CTkCheckBox(self.tab2, text="J6 Open Loop (disable encoder)", variable=self.J6OpenLoopStat, width=200)
        self.J6OpenLoopCbut.place(x=665, y=190)

        # Save Button
        self.saveCalBut = ctk.CTkButton(self.tab2, text="SAVE", command=self.SaveAndApplyCalibration, width=140)
        self.saveCalBut.place(x=1150, y=630)

        """ Tab 2 Entry Fields """

        self.comPortEntryField = ctk.CTkEntry(self.tab2, width=40, justify="center")
        self.comPortEntryField.place(x=50, y=114)

        self.com2PortEntryField = ctk.CTkEntry(self.tab2, width=40, justify="center")
        self.com2PortEntryField.place(x=50, y=184)

        self.com3PortEntryField = ctk.CTkEntry(self.tab2, width=40, justify="center")
        self.com3PortEntryField.place(x=50, y=254)

        self.com3charPortEntryField = ctk.CTkEntry(self.tab2, width=40, justify="center")
        self.com3charPortEntryField.place(x=50, y=284)

        self.com3outPortEntryField = ctk.CTkEntry(self.tab2, width=250, justify="center")
        self.com3outPortEntryField.place(x=50, y=354)

        self.cmdSentEntryField = ctk.CTkEntry(self.tab2, width=950, justify="center")
        self.cmdSentEntryField.place(x=10, y=585)

        self.cmdRecEntryField = ctk.CTkEntry(self.tab2, width=950, justify="center")
        self.cmdRecEntryField.place(x=10, y=645)

        self.J1calOffEntryField = ctk.CTkEntry(self.tab2, width=80, justify="center")
        self.J1calOffEntryField.place(x=540, y=90)

        self.J2calOffEntryField = ctk.CTkEntry(self.tab2, width=80, justify="center")
        self.J2calOffEntryField.place(x=540, y=120)

        self.J3calOffEntryField = ctk.CTkEntry(self.tab2, width=80, justify="center")
        self.J3calOffEntryField.place(x=540, y=150)

        self.J4calOffEntryField = ctk.CTkEntry(self.tab2, width=80, justify="center")
        self.J4calOffEntryField.place(x=540, y=180)

        self.J5calOffEntryField = ctk.CTkEntry(self.tab2, width=80, justify="center")
        self.J5calOffEntryField.place(x=540, y=210)

        self.J6calOffEntryField = ctk.CTkEntry(self.tab2, width=80, justify="center")
        self.J6calOffEntryField.place(x=540, y=240)

        self.J7calOffEntryField = ctk.CTkEntry(self.tab2, width=80, justify="center")
        self.J7calOffEntryField.place(x=540, y=280)

        self.J8calOffEntryField = ctk.CTkEntry(self.tab2, width=80, justify="center")
        self.J8calOffEntryField.place(x=540, y=310)

        self.J9calOffEntryField = ctk.CTkEntry(self.tab2, width=80, justify="center")
        self.J9calOffEntryField.place(x=540, y=340)

        self.axis7lengthEntryField = ctk.CTkEntry(self.tab2, width=60, justify="center")
        self.axis7lengthEntryField.place(x=750, y=340)

        self.axis7rotEntryField = ctk.CTkEntry(self.tab2, width=60, justify="center")
        self.axis7rotEntryField.place(x=750, y=370)

        self.axis7stepsEntryField = ctk.CTkEntry(self.tab2, width=60, justify="center")
        self.axis7stepsEntryField.place(x=750, y=400)

        self.axis8lengthEntryField = ctk.CTkEntry(self.tab2, width=60, justify="center")
        self.axis8lengthEntryField.place(x=950, y=340)

        self.axis8rotEntryField = ctk.CTkEntry(self.tab2, width=60, justify="center")
        self.axis8rotEntryField.place(x=950, y=370)

        self.axis8stepsEntryField = ctk.CTkEntry(self.tab2, width=60, justify="center")
        self.axis8stepsEntryField.place(x=950, y=400)

        self.axis9lengthEntryField = ctk.CTkEntry(self.tab2, width=60, justify="center")
        self.axis9lengthEntryField.place(x=1150, y=340)

        self.axis9rotEntryField = ctk.CTkEntry(self.tab2, width=60, justify="center")
        self.axis9rotEntryField.place(x=1150, y=370)

        self.axis9stepsEntryField = ctk.CTkEntry(self.tab2, width=60, justify="center")
        self.axis9stepsEntryField.place(x=1150, y=400)

        ## TAB 3 LABELS ##

        # Tool Frames
        self.ToolFrameLab = ctk.CTkLabel(self.tab3, text="Tool Frame Offset")
        self.ToolFrameLab.place(x=970, y=60)

        self.UFxLab = ctk.CTkLabel(self.tab3, font=("Arial", 11), text="X")
        self.UFxLab.place(x=920, y=90)

        self.UFyLab = ctk.CTkLabel(self.tab3, font=("Arial", 11), text="Y")
        self.UFyLab.place(x=960, y=90)

        self.UFzLab = ctk.CTkLabel(self.tab3, font=("Arial", 11), text="Z")
        self.UFzLab.place(x=1000, y=90)

        self.UFRxLab = ctk.CTkLabel(self.tab3, font=("Arial", 11), text="Rz")
        self.UFRxLab.place(x=1040, y=90)

        self.UFRyLab = ctk.CTkLabel(self.tab3, font=("Arial", 11), text="Ry")
        self.UFRyLab.place(x=1080, y=90)

        self.UFRzLab = ctk.CTkLabel(self.tab3, font=("Arial", 11), text="Rx")
        self.UFRzLab.place(x=1120, y=90)

        self.TFxEntryField = ctk.CTkEntry(self.tab3, width=50, justify="center")
        self.TFxEntryField.place(x=910, y=115)

        self.TFyEntryField = ctk.CTkEntry(self.tab3, width=50, justify="center")
        self.TFyEntryField.place(x=950, y=115)

        self.TFzEntryField = ctk.CTkEntry(self.tab3, width=50, justify="center")
        self.TFzEntryField.place(x=990, y=115)

        self.TFrzEntryField = ctk.CTkEntry(self.tab3, width=50, justify="center")
        self.TFrzEntryField.place(x=1030, y=115)

        self.TFryEntryField = ctk.CTkEntry(self.tab3, width=50, justify="center")
        self.TFryEntryField.place(x=1070, y=115)

        self.TFrxEntryField = ctk.CTkEntry(self.tab3, width=50, justify="center")
        self.TFrxEntryField.place(x=1110, y=115)

        self.DisableWristCbut = ctk.CTkCheckBox(
            self.tab3, text="Disable Wrist Rotation - Linear Moves", variable=self.DisableWristRot
        )
        self.DisableWristCbut.place(x=910, y=150)

        # Motor Direction Labels and Entry Fields
        self.J1MotDirLab = ctk.CTkLabel(self.tab3, font=("Arial", 8), text="J1 Motor Direction")
        self.J1MotDirLab.place(x=10, y=20)

        self.J2MotDirLab = ctk.CTkLabel(self.tab3, font=("Arial", 8), text="J2 Motor Direction")
        self.J2MotDirLab.place(x=10, y=45)

        self.J3MotDirLab = ctk.CTkLabel(self.tab3, font=("Arial", 8), text="J3 Motor Direction")
        self.J3MotDirLab.place(x=10, y=70)

        self.J4MotDirLab = ctk.CTkLabel(self.tab3, font=("Arial", 8), text="J4 Motor Direction")
        self.J4MotDirLab.place(x=10, y=95)

        self.J5MotDirLab = ctk.CTkLabel(self.tab3, font=("Arial", 8), text="J5 Motor Direction")
        self.J5MotDirLab.place(x=10, y=120)

        self.J6MotDirLab = ctk.CTkLabel(self.tab3, font=("Arial", 8), text="J6 Motor Direction")
        self.J6MotDirLab.place(x=10, y=145)

        self.J7MotDirLab = ctk.CTkLabel(self.tab3, font=("Arial", 8), text="J7 Motor Direction")
        self.J7MotDirLab.place(x=10, y=170)

        self.J8MotDirLab = ctk.CTkLabel(self.tab3, font=("Arial", 8), text="J8 Motor Direction")
        self.J8MotDirLab.place(x=10, y=195)

        self.J9MotDirLab = ctk.CTkLabel(self.tab3, font=("Arial", 8), text="J9 Motor Direction")
        self.J9MotDirLab.place(x=10, y=220)

        self.J1MotDirEntryField = ctk.CTkEntry(self.tab3, width=80, justify="center")
        self.J1MotDirEntryField.place(x=110, y=20)

        self.J2MotDirEntryField = ctk.CTkEntry(self.tab3, width=80, justify="center")
        self.J2MotDirEntryField.place(x=110, y=45)

        self.J3MotDirEntryField = ctk.CTkEntry(self.tab3, width=80, justify="center")
        self.J3MotDirEntryField.place(x=110, y=70)

        self.J4MotDirEntryField = ctk.CTkEntry(self.tab3, width=80, justify="center")
        self.J4MotDirEntryField.place(x=110, y=95)

        self.J5MotDirEntryField = ctk.CTkEntry(self.tab3, width=80, justify="center")
        self.J5MotDirEntryField.place(x=110, y=120)

        self.J6MotDirEntryField = ctk.CTkEntry(self.tab3, width=80, justify="center")
        self.J6MotDirEntryField.place(x=110, y=145)

        self.J7MotDirEntryField = ctk.CTkEntry(self.tab3, width=80, justify="center")
        self.J7MotDirEntryField.place(x=110, y=170)

        self.J8MotDirEntryField = ctk.CTkEntry(self.tab3, width=80, justify="center")
        self.J8MotDirEntryField.place(x=110, y=195)

        self.J9MotDirEntryField = ctk.CTkEntry(self.tab3, width=80, justify="center")
        self.J9MotDirEntryField.place(x=110, y=220)

        # Calibration Directions Labels and Entry Fields
        self.J1CalDirLab = ctk.CTkLabel(self.tab3, font=("Arial", 8), text="J1 Calibration Dir.")
        self.J1CalDirLab.place(x=10, y=280)

        self.J2CalDirLab = ctk.CTkLabel(self.tab3, font=("Arial", 8), text="J2 Calibration Dir.")
        self.J2CalDirLab.place(x=10, y=305)

        self.J3CalDirLab = ctk.CTkLabel(self.tab3, font=("Arial", 8), text="J3 Calibration Dir.")
        self.J3CalDirLab.place(x=10, y=330)

        self.J4CalDirLab = ctk.CTkLabel(self.tab3, font=("Arial", 8), text="J4 Calibration Dir.")
        self.J4CalDirLab.place(x=10, y=355)

        self.J5CalDirLab = ctk.CTkLabel(self.tab3, font=("Arial", 8), text="J5 Calibration Dir.")
        self.J5CalDirLab.place(x=10, y=380)

        self.J6CalDirLab = ctk.CTkLabel(self.tab3, font=("Arial", 8), text="J6 Calibration Dir.")
        self.J6CalDirLab.place(x=10, y=405)

        self.J7CalDirLab = ctk.CTkLabel(self.tab3, font=("Arial", 8), text="J7 Calibration Dir.")
        self.J7CalDirLab.place(x=10, y=430)

        self.J8CalDirLab = ctk.CTkLabel(self.tab3, font=("Arial", 8), text="J8 Calibration Dir.")
        self.J8CalDirLab.place(x=10, y=455)

        self.J9CalDirLab = ctk.CTkLabel(self.tab3, font=("Arial", 8), text="J9 Calibration Dir.")
        self.J9CalDirLab.place(x=10, y=480)

        self.J1CalDirEntryField = ctk.CTkEntry(self.tab3, width=80, justify="center")
        self.J1CalDirEntryField.place(x=110, y=280)

        self.J2CalDirEntryField = ctk.CTkEntry(self.tab3, width=80, justify="center")
        self.J2CalDirEntryField.place(x=110, y=305)

        self.J3CalDirEntryField = ctk.CTkEntry(self.tab3, width=80, justify="center")
        self.J3CalDirEntryField.place(x=110, y=330)

        self.J4CalDirEntryField = ctk.CTkEntry(self.tab3, width=80, justify="center")
        self.J4CalDirEntryField.place(x=110, y=355)

        self.J5CalDirEntryField = ctk.CTkEntry(self.tab3, width=80, justify="center")
        self.J5CalDirEntryField.place(x=110, y=380)

        self.J6CalDirEntryField = ctk.CTkEntry(self.tab3, width=80, justify="center")
        self.J6CalDirEntryField.place(x=110, y=405)

        self.J7CalDirEntryField = ctk.CTkEntry(self.tab3, width=80, justify="center")
        self.J7CalDirEntryField.place(x=110, y=430)

        self.J8CalDirEntryField = ctk.CTkEntry(self.tab3, width=80, justify="center")
        self.J8CalDirEntryField.place(x=110, y=455)

        self.J9CalDirEntryField = ctk.CTkEntry(self.tab3, width=80, justify="center")
        self.J9CalDirEntryField.place(x=110, y=480)

        # Axis Limits Labels and Entry Fields
        self.J1PosLimLab = ctk.CTkLabel(self.tab3, font=("Arial", 8), text="J1 Pos Limit")
        self.J1PosLimLab.place(x=200, y=20)
        self.J1NegLimLab = ctk.CTkLabel(self.tab3, font=("Arial", 8), text="J1 Neg Limit")
        self.J1NegLimLab.place(x=200, y=45)

        self.J2PosLimLab = ctk.CTkLabel(self.tab3, font=("Arial", 8), text="J2 Pos Limit")
        self.J2PosLimLab.place(x=200, y=70)
        self.J2NegLimLab = ctk.CTkLabel(self.tab3, font=("Arial", 8), text="J2 Neg Limit")
        self.J2NegLimLab.place(x=200, y=95)

        self.J3PosLimLab = ctk.CTkLabel(self.tab3, font=("Arial", 8), text="J3 Pos Limit")
        self.J3PosLimLab.place(x=200, y=120)
        self.J3NegLimLab = ctk.CTkLabel(self.tab3, font=("Arial", 8), text="J3 Neg Limit")
        self.J3NegLimLab.place(x=200, y=145)

        self.J4PosLimLab = ctk.CTkLabel(self.tab3, font=("Arial", 8), text="J4 Pos Limit")
        self.J4PosLimLab.place(x=200, y=170)
        self.J4NegLimLab = ctk.CTkLabel(self.tab3, font=("Arial", 8), text="J4 Neg Limit")
        self.J4NegLimLab.place(x=200, y=195)

        self.J5PosLimLab = ctk.CTkLabel(self.tab3, font=("Arial", 8), text="J5 Pos Limit")
        self.J5PosLimLab.place(x=200, y=220)
        self.J5NegLimLab = ctk.CTkLabel(self.tab3, font=("Arial", 8), text="J5 Neg Limit")
        self.J5NegLimLab.place(x=200, y=245)

        self.J6PosLimLab = ctk.CTkLabel(self.tab3, font=("Arial", 8), text="J6 Pos Limit")
        self.J6PosLimLab.place(x=200, y=270)
        self.J6NegLimLab = ctk.CTkLabel(self.tab3, font=("Arial", 8), text="J6 Neg Limit")
        self.J6NegLimLab.place(x=200, y=295)

        self.J1PosLimEntryField = ctk.CTkEntry(self.tab3, width=80, justify="center")
        self.J1PosLimEntryField.place(x=280, y=20)
        self.J1NegLimEntryField = ctk.CTkEntry(self.tab3, width=80, justify="center")
        self.J1NegLimEntryField.place(x=280, y=45)

        self.J2PosLimEntryField = ctk.CTkEntry(self.tab3, width=80, justify="center")
        self.J2PosLimEntryField.place(x=280, y=70)
        self.J2NegLimEntryField = ctk.CTkEntry(self.tab3, width=80, justify="center")
        self.J2NegLimEntryField.place(x=280, y=95)

        self.J3PosLimEntryField = ctk.CTkEntry(self.tab3, width=80, justify="center")
        self.J3PosLimEntryField.place(x=280, y=120)
        self.J3NegLimEntryField = ctk.CTkEntry(self.tab3, width=80, justify="center")
        self.J3NegLimEntryField.place(x=280, y=145)

        self.J4PosLimEntryField = ctk.CTkEntry(self.tab3, width=80, justify="center")
        self.J4PosLimEntryField.place(x=280, y=170)
        self.J4NegLimEntryField = ctk.CTkEntry(self.tab3, width=80, justify="center")
        self.J4NegLimEntryField.place(x=280, y=195)

        self.J5PosLimEntryField = ctk.CTkEntry(self.tab3, width=80, justify="center")
        self.J5PosLimEntryField.place(x=280, y=220)
        self.J5NegLimEntryField = ctk.CTkEntry(self.tab3, width=80, justify="center")
        self.J5NegLimEntryField.place(x=280, y=245)

        self.J6PosLimEntryField = ctk.CTkEntry(self.tab3, width=80, justify="center")
        self.J6PosLimEntryField.place(x=280, y=270)
        self.J6NegLimEntryField = ctk.CTkEntry(self.tab3, width=80, justify="center")
        self.J6NegLimEntryField.place(x=280, y=295)

        # Steps per Degree Labels and Entry Fields
        self.J1StepDegLab = ctk.CTkLabel(self.tab3, font=("Arial", 8), text="J1 Step/Deg")
        self.J1StepDegLab.place(x=200, y=345)

        self.J2StepDegLab = ctk.CTkLabel(self.tab3, font=("Arial", 8), text="J2 Step/Deg")
        self.J2StepDegLab.place(x=200, y=370)

        self.J3StepDegLab = ctk.CTkLabel(self.tab3, font=("Arial", 8), text="J3 Step/Deg")
        self.J3StepDegLab.place(x=200, y=395)

        self.J4StepDegLab = ctk.CTkLabel(self.tab3, font=("Arial", 8), text="J4 Step/Deg")
        self.J4StepDegLab.place(x=200, y=420)

        self.J5StepDegLab = ctk.CTkLabel(self.tab3, font=("Arial", 8), text="J5 Step/Deg")
        self.J5StepDegLab.place(x=200, y=445)

        self.J6StepDegLab = ctk.CTkLabel(self.tab3, font=("Arial", 8), text="J6 Step/Deg")
        self.J6StepDegLab.place(x=200, y=470)

        self.J1StepDegEntryField = ctk.CTkEntry(self.tab3, width=80, justify="center")
        self.J1StepDegEntryField.place(x=280, y=345)

        self.J2StepDegEntryField = ctk.CTkEntry(self.tab3, width=80, justify="center")
        self.J2StepDegEntryField.place(x=280, y=370)

        self.J3StepDegEntryField = ctk.CTkEntry(self.tab3, width=80, justify="center")
        self.J3StepDegEntryField.place(x=280, y=395)

        self.J4StepDegEntryField = ctk.CTkEntry(self.tab3, width=80, justify="center")
        self.J4StepDegEntryField.place(x=280, y=420)

        self.J5StepDegEntryField = ctk.CTkEntry(self.tab3, width=80, justify="center")
        self.J5StepDegEntryField.place(x=280, y=445)

        self.J6StepDegEntryField = ctk.CTkEntry(self.tab3, width=80, justify="center")
        self.J6StepDegEntryField.place(x=280, y=470)

        # Driver Steps Labels and Entry Fields
        self.J1DriveMSLab = ctk.CTkLabel(self.tab3, font=("Arial", 8), text="J1 Drive Microstep")
        self.J1DriveMSLab.place(x=390, y=20)

        self.J2DriveMSLab = ctk.CTkLabel(self.tab3, font=("Arial", 8), text="J2 Drive Microstep")
        self.J2DriveMSLab.place(x=390, y=45)

        self.J3DriveMSLab = ctk.CTkLabel(self.tab3, font=("Arial", 8), text="J3 Drive Microstep")
        self.J3DriveMSLab.place(x=390, y=70)

        self.J4DriveMSLab = ctk.CTkLabel(self.tab3, font=("Arial", 8), text="J4 Drive Microstep")
        self.J4DriveMSLab.place(x=390, y=95)

        self.J5DriveMSLab = ctk.CTkLabel(self.tab3, font=("Arial", 8), text="J5 Drive Microstep")
        self.J5DriveMSLab.place(x=390, y=120)

        self.J6DriveMSLab = ctk.CTkLabel(self.tab3, font=("Arial", 8), text="J6 Drive Microstep")
        self.J6DriveMSLab.place(x=390, y=145)

        self.J1DriveMSEntryField = ctk.CTkEntry(self.tab3, width=80, justify="center")
        self.J1DriveMSEntryField.place(x=500, y=20)

        self.J2DriveMSEntryField = ctk.CTkEntry(self.tab3, width=80, justify="center")
        self.J2DriveMSEntryField.place(x=500, y=45)

        self.J3DriveMSEntryField = ctk.CTkEntry(self.tab3, width=80, justify="center")
        self.J3DriveMSEntryField.place(x=500, y=70)

        self.J4DriveMSEntryField = ctk.CTkEntry(self.tab3, width=80, justify="center")
        self.J4DriveMSEntryField.place(x=500, y=95)

        self.J5DriveMSEntryField = ctk.CTkEntry(self.tab3, width=80, justify="center")
        self.J5DriveMSEntryField.place(x=500, y=120)

        self.J6DriveMSEntryField = ctk.CTkEntry(self.tab3, width=80, justify="center")
        self.J6DriveMSEntryField.place(x=500, y=145)

        # Encoder CPR Labels and Entry Fields
        self.J1EncCPRLab = ctk.CTkLabel(self.tab3, font=("Arial", 8), text="J1 Encoder CPR")
        self.J1EncCPRLab.place(x=390, y=195)

        self.J2EncCPRLab = ctk.CTkLabel(self.tab3, font=("Arial", 8), text="J2 Encoder CPR")
        self.J2EncCPRLab.place(x=390, y=220)

        self.J3EncCPRLab = ctk.CTkLabel(self.tab3, font=("Arial", 8), text="J3 Encoder CPR")
        self.J3EncCPRLab.place(x=390, y=245)

        self.J4EncCPRLab = ctk.CTkLabel(self.tab3, font=("Arial", 8), text="J4 Encoder CPR")
        self.J4EncCPRLab.place(x=390, y=270)

        self.J5EncCPRLab = ctk.CTkLabel(self.tab3, font=("Arial", 8), text="J5 Encoder CPR")
        self.J5EncCPRLab.place(x=390, y=295)

        self.J6EncCPRLab = ctk.CTkLabel(self.tab3, font=("Arial", 8), text="J6 Encoder CPR")
        self.J6EncCPRLab.place(x=390, y=320)

        self.J1EncCPREntryField = ctk.CTkEntry(self.tab3, width=80, justify="center")
        self.J1EncCPREntryField.place(x=500, y=195)

        self.J2EncCPREntryField = ctk.CTkEntry(self.tab3, width=80, justify="center")
        self.J2EncCPREntryField.place(x=500, y=220)

        self.J3EncCPREntryField = ctk.CTkEntry(self.tab3, width=80, justify="center")
        self.J3EncCPREntryField.place(x=500, y=245)

        self.J4EncCPREntryField = ctk.CTkEntry(self.tab3, width=80, justify="center")
        self.J4EncCPREntryField.place(x=500, y=270)

        self.J5EncCPREntryField = ctk.CTkEntry(self.tab3, width=80, justify="center")
        self.J5EncCPREntryField.place(x=500, y=295)

        self.J6EncCPREntryField = ctk.CTkEntry(self.tab3, width=80, justify="center")
        self.J6EncCPREntryField.place(x=500, y=320)

        # DH Parameters Labels
        self.JointLabels = []
        for i in range(1, 7):
            joint_label = ctk.CTkLabel(self.tab3, font=("Arial", 8), text=f"J{i}")
            joint_label.place(x=600, y=25 + i * 25)
            self.JointLabels.append(joint_label)

        self.DHParamLabels = {
            "Θ": ctk.CTkLabel(self.tab3, font=("Arial", 8), text="DH-Θ"),
            "α": ctk.CTkLabel(self.tab3, font=("Arial", 8), text="DH-α"),
            "d": ctk.CTkLabel(self.tab3, font=("Arial", 8), text="DH-d"),
            "a": ctk.CTkLabel(self.tab3, font=("Arial", 8), text="DH-a")
        }
        self.DHParamLabels["Θ"].place(x=645, y=20)
        self.DHParamLabels["α"].place(x=700, y=20)
        self.DHParamLabels["d"].place(x=755, y=20)
        self.DHParamLabels["a"].place(x=810, y=20)

        # DH Parameters Entry Fields
        self.DHEntryFields = {
            "Θ": [],
            "α": [],
            "d": [],
            "a": []
        }

        for i in range(6):  # J1 to J6
            for param, x_offset in zip(["Θ", "α", "d", "a"], [630, 685, 740, 795]):
                entry_field = ctk.CTkEntry(self.tab3, width=80, justify="center")
                entry_field.place(x=x_offset, y=45 + i * 25)
                self.DHEntryFields[param].append(entry_field)

        # Load Default Profiles
        self.loadAR4Mk2But = ctk.CTkButton(self.tab3, text="Load AR4-MK3 Defaults", width=200, command=self.LoadAR4Mk3default)
        self.loadAR4Mk2But.place(x=1150, y=590)

        # Save Cablibration Profile
        self.saveCalBut = ctk.CTkButton(self.tab3, text="SAVE", width=200, command=self.SaveAndApplyCalibration)
        self.saveCalBut.place(x=1150, y=630)

        ## TAB 4 LABELS ##

        # Servo Labels
        self.servo0onequalsLab = ctk.CTkLabel(self.tab4, text="=")
        self.servo0onequalsLab.place(x=76, y=12)

        self.servo0offequalsLab = ctk.CTkLabel(self.tab4, text="=")
        self.servo0offequalsLab.place(x=76, y=52)

        self.servo1onequalsLab = ctk.CTkLabel(self.tab4, text="=")
        self.servo1onequalsLab.place(x=76, y=92)

        self.servo1offequalsLab = ctk.CTkLabel(self.tab4, text="=")
        self.servo1offequalsLab.place(x=76, y=132)

        self.servo2onequalsLab = ctk.CTkLabel(self.tab4, text="=")
        self.servo2onequalsLab.place(x=76, y=172)

        self.servo2offequalsLab = ctk.CTkLabel(self.tab4, text="=")
        self.servo2offequalsLab.place(x=76, y=212)

        self.servo3onequalsLab = ctk.CTkLabel(self.tab4, text="=")
        self.servo3onequalsLab.place(x=76, y=252)

        self.servo3offequalsLab = ctk.CTkLabel(self.tab4, text="=")
        self.servo3offequalsLab.place(x=76, y=292)

        # Digital Output Labels
        self.Do1onequalsLab = ctk.CTkLabel(self.tab4, text="=")
        self.Do1onequalsLab.place(x=222, y=12)

        self.Do1offequalsLab = ctk.CTkLabel(self.tab4, text="=")
        self.Do1offequalsLab.place(x=222, y=52)

        self.Do2onequalsLab = ctk.CTkLabel(self.tab4, text="=")
        self.Do2onequalsLab.place(x=222, y=92)

        self.Do2offequalsLab = ctk.CTkLabel(self.tab4, text="=")
        self.Do2offequalsLab.place(x=222, y=132)

        self.Do3onequalsLab = ctk.CTkLabel(self.tab4, text="=")
        self.Do3onequalsLab.place(x=222, y=172)

        self.Do3offequalsLab = ctk.CTkLabel(self.tab4, text="=")
        self.Do3offequalsLab.place(x=222, y=212)

        self.Do4onequalsLab = ctk.CTkLabel(self.tab4, text="=")
        self.Do4onequalsLab.place(x=222, y=252)

        self.Do4offequalsLab = ctk.CTkLabel(self.tab4, text="=")
        self.Do4offequalsLab.place(x=222, y=292)

        self.Do5onequalsLab = ctk.CTkLabel(self.tab4, text="=")
        self.Do5onequalsLab.place(x=222, y=332)

        self.Do5offequalsLab = ctk.CTkLabel(self.tab4, text="=")
        self.Do5offequalsLab.place(x=222, y=372)

        self.Do6onequalsLab = ctk.CTkLabel(self.tab4, text="=")
        self.Do6onequalsLab.place(x=222, y=412)

        self.Do6offequalsLab = ctk.CTkLabel(self.tab4, text="=")
        self.Do6offequalsLab.place(x=222, y=452)

        # Note Labels
        self.inoutavailLab1 = ctk.CTkLabel(
            self.tab4,
            text="NOTE: the following are available when using the default Nano board for IO:   Inputs = 2-7  /  Outputs = 8-13  /  Servos = A0-A7",
            wraplength=800,
            justify="left",
        )
        self.inoutavailLab1.place(x=10, y=640)

        self.inoutavailLab2 = ctk.CTkLabel(
            self.tab4,
            text="If using IO on Teensy board:  Inputs = 32-36  /  Outputs = 37-41 - if using IO on Teensy you must manually change the command from 'Out On =' to 'ToutOn ='",
            wraplength=800,
            justify="left",
        )
        self.inoutavailLab2.place(x=10, y=662)

        # Servo Buttons
        self.servo0onBut = ctk.CTkButton(self.tab4, text="Servo 0", width=50, command=self.Servo0on)
        self.servo0onBut.place(x=10, y=10)

        self.servo0offBut = ctk.CTkButton(self.tab4, text="Servo 0", width=50, command=self.Servo0off)
        self.servo0offBut.place(x=10, y=50)

        self.servo1onBut = ctk.CTkButton(self.tab4, text="Servo 1", width=50, command=self.Servo1on)
        self.servo1onBut.place(x=10, y=90)

        self.servo1offBut = ctk.CTkButton(self.tab4, text="Servo 1", width=50, command=self.Servo1off)
        self.servo1offBut.place(x=10, y=130)

        self.servo2onBut = ctk.CTkButton(self.tab4, text="Servo 2", width=50, command=self.Servo2on)
        self.servo2onBut.place(x=10, y=170)

        self.servo2offBut = ctk.CTkButton(self.tab4, text="Servo 2", width=50, command=self.Servo2off)
        self.servo2offBut.place(x=10, y=210)

        self.servo3onBut = ctk.CTkButton(self.tab4, text="Servo 3", width=50, command=self.Servo3on)
        self.servo3onBut.place(x=10, y=250)

        self.servo3offBut = ctk.CTkButton(self.tab4, text="Servo 3", width=50, command=self.Servo3off)
        self.servo3offBut.place(x=10, y=290)

        # Digital Output Buttons
        self.DO1onBut = ctk.CTkButton(self.tab4, text="DO on", width=50, command=self.DO1on)
        self.DO1onBut.place(x=164, y=10)

        self.DO1offBut = ctk.CTkButton(self.tab4, text="DO off", width=50, command=self.DO1off)
        self.DO1offBut.place(x=164, y=50)

        self.DO2onBut = ctk.CTkButton(self.tab4, text="DO on", width=50, command=self.DO2on)
        self.DO2onBut.place(x=164, y=90)

        self.DO2offBut = ctk.CTkButton(self.tab4, text="DO off", width=50, command=self.DO2off)
        self.DO2offBut.place(x=164, y=130)

        self.DO3onBut = ctk.CTkButton(self.tab4, text="DO on", width=50, command=self.DO3on)
        self.DO3onBut.place(x=164, y=170)

        self.DO3offBut = ctk.CTkButton(self.tab4, text="DO off", width=50, command=self.DO3off)
        self.DO3offBut.place(x=164, y=210)

        self.DO4onBut = ctk.CTkButton(self.tab4, text="DO on", width=50, command=self.DO4on)
        self.DO4onBut.place(x=164, y=250)

        self.DO4offBut = ctk.CTkButton(self.tab4, text="DO off", width=50, command=self.DO4off)
        self.DO4offBut.place(x=164, y=290)

        self.DO5onBut = ctk.CTkButton(self.tab4, text="DO on", width=50, command=self.DO5on)
        self.DO5onBut.place(x=164, y=330)

        self.DO5offBut = ctk.CTkButton(self.tab4, text="DO off", width=50, command=self.DO5off)
        self.DO5offBut.place(x=164, y=370)

        self.DO6onBut = ctk.CTkButton(self.tab4, text="DO on",width=50, command=self.DO6on)
        self.DO6onBut.place(x=164, y=410)

        self.DO6offBut = ctk.CTkButton(self.tab4, text="DO off", width=50, command=self.DO6off)
        self.DO6offBut.place(x=164, y=450)

        # Servo Entry Fields
        self.servo0onEntryField = ctk.CTkEntry(self.tab4, width=50, justify="center")
        self.servo0onEntryField.place(x=90, y=15)

        self.servo0offEntryField = ctk.CTkEntry(self.tab4, width=50, justify="center")
        self.servo0offEntryField.place(x=90, y=55)

        self.servo1onEntryField = ctk.CTkEntry(self.tab4, width=50, justify="center")
        self.servo1onEntryField.place(x=90, y=95)

        self.servo1offEntryField = ctk.CTkEntry(self.tab4, width=50, justify="center")
        self.servo1offEntryField.place(x=90, y=135)

        self.servo2onEntryField = ctk.CTkEntry(self.tab4, width=50, justify="center")
        self.servo2onEntryField.place(x=90, y=175)

        self.servo2offEntryField = ctk.CTkEntry(self.tab4, width=50, justify="center")
        self.servo2offEntryField.place(x=90, y=215)

        self.servo3onEntryField = ctk.CTkEntry(self.tab4, width=50, justify="center")
        self.servo3onEntryField.place(x=90, y=255)

        self.servo3offEntryField = ctk.CTkEntry(self.tab4, width=50, justify="center")
        self.servo3offEntryField.place(x=90, y=295)

        # Digital Output Entry Fields
        self.DO1onEntryField = ctk.CTkEntry(self.tab4, width=50, justify="center")
        self.DO1onEntryField.place(x=238, y=15)

        self.DO1offEntryField = ctk.CTkEntry(self.tab4, width=50, justify="center")
        self.DO1offEntryField.place(x=238, y=55)

        self.DO2onEntryField = ctk.CTkEntry(self.tab4, width=50, justify="center")
        self.DO2onEntryField.place(x=238, y=95)

        self.DO2offEntryField = ctk.CTkEntry(self.tab4, width=50, justify="center")
        self.DO2offEntryField.place(x=238, y=135)

        self.DO3onEntryField = ctk.CTkEntry(self.tab4, width=50, justify="center")
        self.DO3onEntryField.place(x=238, y=175)

        self.DO3offEntryField = ctk.CTkEntry(self.tab4, width=50, justify="center")
        self.DO3offEntryField.place(x=238, y=215)

        self.DO4onEntryField = ctk.CTkEntry(self.tab4, width=50, justify="center")
        self.DO4onEntryField.place(x=238, y=255)

        self.DO4offEntryField = ctk.CTkEntry(self.tab4, width=50, justify="center")
        self.DO4offEntryField.place(x=238, y=295)

        self.DO5onEntryField = ctk.CTkEntry(self.tab4, width=50, justify="center")
        self.DO5onEntryField.place(x=238, y=335)

        self.DO5offEntryField = ctk.CTkEntry(self.tab4, width=50, justify="center")
        self.DO5offEntryField.place(x=238, y=375)

        self.DO6onEntryField = ctk.CTkEntry(self.tab4, width=50, justify="center")
        self.DO6onEntryField.place(x=238, y=415)

        self.DO6offEntryField = ctk.CTkEntry(self.tab4, width=50, justify="center")
        self.DO6offEntryField.place(x=238, y=455)

        ## TAB 5 LABELS ##

        # Labels for R1-R16
        self.R_labels = []
        for i in range(16):
            label = ctk.CTkLabel(self.tab5, text=f"R{i+1}")
            label.place(x=86, y=30 + i * 30)
            self.R_labels.append(label)

        # Labels for PR1-PR16
        self.PR_labels = []
        for i in range(16):
            label = ctk.CTkLabel(self.tab5, text=f"PR{i+1}")
            label.place(x=656, y=30 + i * 30)
            self.PR_labels.append(label)

        # Labels for SP_E1 to SP_E6
        self.SP_E_labels = []
        SP_E_texts = ["X", "Y", "Z", "Rz", "Ry", "Rx"]
        SP_E_positions = [410, 450, 490, 530, 570, 610]
        for i, (text, x_pos) in enumerate(zip(SP_E_texts, SP_E_positions)):
            label = ctk.CTkLabel(self.tab5, text=text)
            label.place(x=x_pos, y=10)
            self.SP_E_labels.append(label)

        # Entry Fields for R1-R16
        self.R_entry_fields = []
        for i in range(16):
            entry = ctk.CTkEntry(self.tab5, width=50, justify="center")
            entry.place(x=30, y=30 + i * 30)
            self.R_entry_fields.append(entry)

        # Create SP fields dynamically for E1, E2, ..., E6
        self.SP_entry_fields = {}  # Dictionary to hold all SP entry fields by column
        x_coords = [400, 440, 480, 520, 560, 600]  # X-coordinates for columns E1 to E6

        for col, x in enumerate(x_coords, start=1):  # Start column numbering from 1 (E1, E2, ...)
            self.SP_entry_fields[f"E{col}"] = []
            for i in range(16):
                entry = ctk.CTkEntry(self.tab5, width=50, justify="center")
                entry.place(x=x, y=30 + i * 30)
                self.SP_entry_fields[f"E{col}"].append(entry)

        ## TAB 6 LABELS ##

        # Visual Backdrop Label
        self.VisBackdromLbl = ctk.CTkLabel(
            self.tab6, 
            image=ctk.CTkImage(Image.open(os.path.join('assets', 'VisBackdrop.png')), size=(690,530)), 
            text=""
        )
        self.VisBackdromLbl.place(x=15, y=215)

        # Video Frame
        self.video_frame = ctk.CTkFrame(self.tab6, width=640, height=480)
        self.video_frame.place(x=50, y=250)

        self.vid_lbl = ctk.CTkLabel(self.video_frame, text="")
        self.vid_lbl.place(x=0, y=0)
        self.vid_lbl.bind('<Button-1>', self.motion)

        # Live Video Feed
        self.LiveLab = ctk.CTkLabel(self.tab6, text="LIVE VIDEO FEED", font=("Arial", 12))
        self.LiveLab.place(x=750, y=390)

        self.liveCanvas = ctk.CTkCanvas(self.tab6, width=490, height=330)
        self.liveCanvas.place(x=750, y=410)

        self.live_frame = ctk.CTkFrame(self.tab6, width=480, height=320)
        self.live_frame.place(x=757, y=417)

        self.live_lbl = ctk.CTkLabel(self.live_frame, text="")
        self.live_lbl.place(x=0, y=0)

        # Template Frame
        self.template_frame = ctk.CTkFrame(self.tab6, width=150, height=150)
        self.template_frame.place(x=575, y=50)

        self.template_lbl = ctk.CTkLabel(self.template_frame, text="")
        self.template_lbl.place(x=0, y=0)

        # Found Values and Calibration Values Labels
        self.FoundValuesLab = ctk.CTkLabel(self.tab6, text="FOUND VALUES", font=("Arial", 12))
        self.FoundValuesLab.place(x=750, y=30)

        self.CalValuesLab = ctk.CTkLabel(self.tab6, text="CALIBRATION VALUES", font=("Arial", 12))
        self.CalValuesLab.place(x=900, y=30)

        # Camera Selection Menu
        self.graph = FilterGraph()

        try:
            self.camList = self.graph.get_input_devices()
        except:
            self.camList = ["Select a Camera"]

        self.visoptions = ctk.StringVar(value="Select a Camera")

        try:
            self.vismenu = ctk.CTkOptionMenu(self.tab6, variable=self.visoptions, values=self.camList)
            self.vismenu.configure(width=200)  # Width in pixels for CTkOptionMenu
            self.vismenu.place(x=10, y=10)
        except Exception as e:
            print("No camera found:", e)

        # Buttons
        self.StartCamBut = ctk.CTkButton(self.tab6, text="Start Camera", width=140, command=self.start_vid)
        self.StartCamBut.place(x=200, y=10)

        self.StopCamBut = ctk.CTkButton(self.tab6, text="Stop Camera", width=140, command=self.stop_vid)
        self.StopCamBut.place(x=315, y=10)

        self.CapImgBut = ctk.CTkButton(self.tab6, text="Snap Image", width=140, command=self.take_pic)
        self.CapImgBut.place(x=10, y=50)

        self.TeachImgBut = ctk.CTkButton(self.tab6, text="Teach Object", width=140, command=self.selectTemplate)
        self.TeachImgBut.place(x=140, y=50)

        self.FindVisBut = ctk.CTkButton(self.tab6, text="Snap & Find", width=140, command=self.snapFind)
        self.FindVisBut.place(x=270, y=50)

        self.ZeroBrCnBut = ctk.CTkButton(self.tab6, text="Zero", width=50, command=self.zeroBrCn)
        self.ZeroBrCnBut.place(x=10, y=110)

        self.MaskBut = ctk.CTkButton(self.tab6, text="Mask", width=50, command=self.selectMask)
        self.MaskBut.place(x=10, y=150)

        # Sliders and Labels
        self.VisZoomSlide = ctk.CTkSlider(self.tab6, from_=50, to=1, width=250, orientation="horizontal", command=self.VisUpdateBriCon)
        self.VisZoomSlide.place(x=75, y=95)
        self.VisZoomSlide.set(50)

        self.VisZoomLab = ctk.CTkLabel(self.tab6, text="Zoom")
        self.VisZoomLab.place(x=75, y=115)

        self.VisBrightSlide = ctk.CTkSlider(self.tab6, from_=-127, to=127, width=250, orientation="horizontal", command=self.VisUpdateBriCon)
        self.VisBrightSlide.place(x=75, y=130)

        self.VisBrightLab = ctk.CTkLabel(self.tab6, text="Brightness")
        self.VisBrightLab.place(x=75, y=150)

        self.VisContrastSlide = ctk.CTkSlider(self.tab6, from_=-127, to=127, width=250, orientation="horizontal", command=self.VisUpdateBriCon)
        self.VisContrastSlide.place(x=75, y=165)

        self.VisContrastLab = ctk.CTkLabel(self.tab6, text="Contrast")
        self.VisContrastLab.place(x=75, y=185)

        # Checkboxes
        self.fullRotCbut = ctk.CTkCheckBox(self.tab6, text="Full Rotation Search", variable=self.fullRot)
        self.fullRotCbut.place(x=900, y=255)

        self.pick180Cbut = ctk.CTkCheckBox(self.tab6, text="Pick Closest 180°", variable=self.pick180)
        self.pick180Cbut.place(x=900, y=275)

        self.pickClosestCbut = ctk.CTkCheckBox(self.tab6, text="Try Closest When Out of Range", variable=self.pickClosest)
        self.pickClosestCbut.place(x=900, y=295)

        # Save Button
        self.SaveCalBut = ctk.CTkButton(self.tab6, text="SAVE VISION DATA", width=200, command=self.SaveAndApplyCalibration)
        self.SaveCalBut.place(x=915, y=340)

        # Entry Fields and Labels
        self.VisBacColorEntryField = ctk.CTkEntry(self.tab6, width=150, justify="center")
        self.VisBacColorEntryField.place(x=390, y=100)
        self.VisBacColorLab = ctk.CTkLabel(self.tab6, text="Background Color")
        self.VisBacColorLab.place(x=390, y=120)

        self.bgAutoCbut = ctk.CTkCheckBox(self.tab6, text="Auto", command=self.checkAutoBG, variable=self.autoBG)
        self.bgAutoCbut.place(x=490, y=101)

        # Score Threshold
        self.VisScoreEntryField = ctk.CTkEntry(self.tab6, width=150, justify="center")
        self.VisScoreEntryField.place(x=390, y=150)
        self.VisScoreLab = ctk.CTkLabel(self.tab6, text="Score Threshold")
        self.VisScoreLab.place(x=390, y=170)

        # Scored Value
        self.VisRetScoreEntryField = ctk.CTkEntry(self.tab6, width=150, justify="center")
        self.VisRetScoreEntryField.place(x=750, y=55)
        self.VisRetScoreLab = ctk.CTkLabel(self.tab6, text="Scored Value")
        self.VisRetScoreLab.place(x=750, y=75)

        # Found Angle
        self.VisRetAngleEntryField = ctk.CTkEntry(self.tab6, width=150, justify="center")
        self.VisRetAngleEntryField.place(x=750, y=105)
        self.VisRetAngleLab = ctk.CTkLabel(self.tab6, text="Found Angle")
        self.VisRetAngleLab.place(x=750, y=125)

        # Pixel Positions
        self.VisRetXpixEntryField = ctk.CTkEntry(self.tab6, width=150, justify="center")
        self.VisRetXpixEntryField.place(x=750, y=155)
        self.VisRetXpixLab = ctk.CTkLabel(self.tab6, text="Pixel X Position")
        self.VisRetXpixLab.place(x=750, y=175)

        self.VisRetYpixEntryField = ctk.CTkEntry(self.tab6, width=150, justify="center")
        self.VisRetYpixEntryField.place(x=750, y=205)
        self.VisRetYpixLab = ctk.CTkLabel(self.tab6, text="Pixel Y Position")
        self.VisRetYpixLab.place(x=750, y=225)

        # Robot Positions
        self.VisRetXrobEntryField = ctk.CTkEntry(self.tab6, width=150, justify="center")
        self.VisRetXrobEntryField.place(x=750, y=255)
        self.VisRetXrobLab = ctk.CTkLabel(self.tab6, text="Robot X Position")
        self.VisRetXrobLab.place(x=750, y=275)

        self.VisRetYrobEntryField = ctk.CTkEntry(self.tab6, width=150, justify="center")
        self.VisRetYrobEntryField.place(x=750, y=305)
        self.VisRetYrobLab = ctk.CTkLabel(self.tab6, text="Robot Y Position")
        self.VisRetYrobLab.place(x=750, y=325)

        # Pixel Position X1, Y1, X2, Y2
        self.VisX1PixEntryField = ctk.CTkEntry(self.tab6, width=150, justify="center")
        self.VisX1PixEntryField.place(x=900, y=55)
        self.VisX1PixLab = ctk.CTkLabel(self.tab6, text="X1 Pixel Pos")
        self.VisX1PixLab.place(x=900, y=75)

        self.VisY1PixEntryField = ctk.CTkEntry(self.tab6, width=150, justify="center")
        self.VisY1PixEntryField.place(x=900, y=105)
        self.VisY1PixLab = ctk.CTkLabel(self.tab6, text="Y1 Pixel Pos")
        self.VisY1PixLab.place(x=900, y=125)

        self.VisX2PixEntryField = ctk.CTkEntry(self.tab6, width=150, justify="center")
        self.VisX2PixEntryField.place(x=900, y=155)
        self.VisX2PixLab = ctk.CTkLabel(self.tab6, text="X2 Pixel Pos")
        self.VisX2PixLab.place(x=900, y=175)

        self.VisY2PixEntryField = ctk.CTkEntry(self.tab6, width=150, justify="center")
        self.VisY2PixEntryField.place(x=900, y=205)
        self.VisY2PixLab = ctk.CTkLabel(self.tab6, text="Y2 Pixel Pos")
        self.VisY2PixLab.place(x=900, y=225)

        # Robot Position X1, Y1, X2, Y2
        self.VisX1RobEntryField = ctk.CTkEntry(self.tab6, width=150, justify="center")
        self.VisX1RobEntryField.place(x=1010, y=55)
        self.VisX1RobLab = ctk.CTkLabel(self.tab6, text="X1 Robot Pos")
        self.VisX1RobLab.place(x=1010, y=75)

        self.VisY1RobEntryField = ctk.CTkEntry(self.tab6, width=150, justify="center")
        self.VisY1RobEntryField.place(x=1010, y=105)
        self.VisY1RobLab = ctk.CTkLabel(self.tab6, text="Y1 Robot Pos")
        self.VisY1RobLab.place(x=1010, y=125)

        self.VisX2RobEntryField = ctk.CTkEntry(self.tab6, width=150, justify="center")
        self.VisX2RobEntryField.place(x=1010, y=155)
        self.VisX2RobLab = ctk.CTkLabel(self.tab6, text="X2 Robot Pos")
        self.VisX2RobLab.place(x=1010, y=175)

        self.VisY2RobEntryField = ctk.CTkEntry(self.tab6, width=150, justify="center")
        self.VisY2RobEntryField.place(x=1010, y=205)
        self.VisY2RobLab = ctk.CTkLabel(self.tab6, text="Y2 Robot Pos")
        self.VisY2RobLab.place(x=1010, y=225)

        ## TAB 7 LABELS ##

        

    # Startup defs #

    def startup(self):
        self.updateParams()
        time.sleep(0.1)

        self.calExtAxis()
        time.sleep(0.1)

        self.sendPos()
        time.sleep(0.1)

        self.requestPos()

    # Communication defs #

    def setCom(self):
        try:
            port = "COM" + self.comPortEntryField.get()
            baud = 9600
            self.ser = serial.Serial(port, baud)

            # Update status labels
            self.almStatusLab.configure(text="SYSTEM READY", style="OK.TLabel")
            self.almStatusLab2.configure(text="SYSTEM READY", style="OK.TLabel")

            # Log success
            Curtime = datetime.datetime.now().strftime("%B %d %Y - %I:%M%p")
            self.tab8.ElogView.insert(
                END, f"{Curtime} - COMMUNICATIONS STARTED WITH TEENSY 4.1 CONTROLLER"
            )
            value = self.tab8.ElogView.get(0, END)
            pickle.dump(value, open("ErrorLog", "wb"))

            time.sleep(0.1)
            self.ser.flushInput()
            self.startup()

        except:
            # Update status labels on failure
            error_message = (
                "UNABLE TO ESTABLISH COMMUNICATIONS WITH TEENSY 4.1 CONTROLLER"
            )
            self.almStatusLab.configure(text=error_message, style="Alarm.TLabel")
            self.almStatusLab2.configure(text=error_message, style="Alarm.TLabel")

            # Log failure
            Curtime = datetime.datetime.now().strftime("%B %d %Y - %I:%M%p")
            self.tab8.ElogView.insert(
                END, f"{Curtime} - {error_message}"
            )
            value = self.tab8.ElogView.get(0, END)
            pickle.dump(value, open("ErrorLog", "wb"))

    def setCom2(self):
        try:
            port = "COM" + self.com2PortEntryField.get()
            baud = 115200
            self.ser2 = serial.Serial(port, baud)

            # Update status labels
            self.almStatusLab.configure(text="SYSTEM READY", style="OK.TLabel")
            self.almStatusLab2.configure(text="SYSTEM READY", style="OK.TLabel")

            # Log success
            Curtime = datetime.datetime.now().strftime("%B %d %Y - %I:%M%p")
            self.tab8.ElogView.insert(
                END, f"{Curtime} - COMMUNICATIONS STARTED WITH ARDUINO IO BOARD"
            )
            value = self.tab8.ElogView.get(0, END)
            pickle.dump(value, open("ErrorLog", "wb"))

        except:
            # Log failure
            error_message = (
                "UNABLE TO ESTABLISH COMMUNICATIONS WITH ARDUINO IO BOARD"
            )
            Curtime = datetime.datetime.now().strftime("%B %d %Y - %I:%M%p")
            self.tab8.ElogView.insert(
                END, f"{Curtime} - {error_message}"
            )
            value = self.tab8.ElogView.get(0, END)
            pickle.dump(value, open("ErrorLog", "wb"))

    def darkTheme(self):
        self.curTheme = 0

        # Set dark mode
        ctk.set_appearance_mode("dark")  # "dark" for dark theme
        ctk.set_default_color_theme("blue")  # Optionally set a specific color theme

        # Update custom styles (if specific widget styles are still needed)
        self.almStatusLab.configure(text_color="IndianRed1", font=("Arial", 10, "bold"))
        self.almStatusLab2.configure(text_color="IndianRed1", font=("Arial", 10, "bold"))

        self.warnLabel.configure(text_color="orange", font=("Arial", 10, "bold"))
        self.okLabel.configure(text_color="light green", font=("Arial", 10, "bold"))
        self.jointLimLabel.configure(text_color="light blue", font=("Arial", 8))

    def lightTheme(self):
        self.curTheme = 1

        # Set light mode
        ctk.set_appearance_mode("light")  # "light" for light theme
        ctk.set_default_color_theme("blue")  # Optionally set a specific color theme

        # Update custom styles (if specific widget styles are still needed)
        self.almStatusLab.configure(text_color="red", font=("Arial", 10, "bold"))
        self.almStatusLab2.configure(text_color="red", font=("Arial", 10, "bold"))

        self.warnLabel.configure(text_color="dark orange", font=("Arial", 10, "bold"))
        self.okLabel.configure(text_color="green", font=("Arial", 10, "bold"))
        self.jointLimLabel.configure(text_color="dark blue", font=("Arial", 8))

    # Execution defs #

    def runProg(self):
        def threadProg():
            self.estopActive = False
            self.posOutreach = False
            self.stopQueue = "0"
            self.splineActive = "0"
            try:
                curRow = self.tab1.progView.curselection()[0]
                if curRow == 0:
                    curRow = 1
            except:
                curRow = 1
                self.tab1.progView.selection_clear(0, END)
                self.tab1.progView.select_set(curRow)

            runTrue = True
            while runTrue:
                if not runTrue:
                    if self.estopActive:
                        self.almStatusLab.configure(
                            text="Estop Button was Pressed", fg_color="red")
                    elif self.posOutreach:
                        self.almStatusLab.configure(
                            text="Position Out of Reach", fg_color="red")
                    else:
                        self.almStatusLab.configure(
                            text="PROGRAM STOPPED", fg_color="red")
                else:
                    self.almStatusLab.configure(
                        text="PROGRAM RUNNING", fg_color="green")

                rowInProc = True
                self.executeRow()
                while rowInProc:
                    time.sleep(0.1)

                selRow = self.tab1.progView.curselection()[0]
                last = self.tab1.progView.index('end')
                self.tab1.progView.selection_clear(0, END)
                selRow += 1
                self.tab1.progView.select_set(selRow)
                curRow += 1
                time.sleep(0.1)

                try:
                    selRow = self.tab1.progView.curselection()[
                        0]
                    self.curRowEntryField.delete(0, 'end')
                    self.curRowEntryField.insert(0, selRow)
                except:
                    self.curRowEntryField.delete(0, 'end')
                    self.curRowEntryField.insert(0, "---")
                    runTrue = False
                    if self.estopActive:
                        self.almStatusLab.configure(
                            text="Estop Button was Pressed", fg_color="red")
                    elif self.posOutreach:
                        self.almStatusLab.configure(
                            text="Position Out of Reach", fg_color="red")
                    else:
                        self.almStatusLab.configure(
                            text="PROGRAM STOPPED", fg_color="red")
        t = threading.Thread(target=threadProg)
        t.start()

    def stepFwd(self):
        self.estopActive = False
        self.posOutreach = False
        self.almStatusLab.configure(text="SYSTEM READY", fg_color="green")
        self.executeRow()
        selRow = self.tab1.progView.curselection()[0]
        last = self.tab1.progView.index('end')
        for row in range(0, selRow):
            self.tab1.progView.itemconfig(
                row, {'fg': 'dodger blue'})
        self.tab1.progView.itemconfig(selRow, {'fg': 'blue2'})
        for row in range(selRow + 1, last):
            self.tab1.progView.itemconfig(
                row, {'fg': 'black'})
        self.tab1.progView.selection_clear(0, END)
        selRow += 1
        self.tab1.progView.select_set(selRow)
        try:
            selRow = self.tab1.progView.curselection()[0]
            self.curRowEntryField.delete(0, 'end')
            self.curRowEntryField.insert(0, selRow)
        except:
            self.curRowEntryField.delete(0, 'end')
            self.curRowEntryField.insert(0, "---")

    def stepRev(self):
        self.estopActive = False
        self.posOutreach = False
        self.almStatusLab.configure(text="SYSTEM READY", fg_color="green")
        self.executeRow()
        selRow = self.tab1.progView.curselection()[0]
        last = self.tab1.progView.index('end')
        for row in range(0, selRow):
            self.tab1.progView.itemconfig(
                row, {'fg': 'black'})
        self.tab1.progView.itemconfig(selRow, {'fg': 'red'})
        for row in range(selRow + 1, last):
            self.tab1.progView.itemconfig(
                row, {'fg': 'tomato2'})
        self.tab1.progView.selection_clear(0, END)
        selRow -= 1
        self.tab1.progView.select_set(selRow)
        try:
            selRow = self.tab1.progView.curselection()[0]
            self.curRowEntryField.delete(0, 'end')
            self.curRowEntryField.insert(0, selRow)
        except:
            self.curRowEntryField.delete(0, 'end')
            self.curRowEntryField.insert(0, "---")

    def stopProg(self):
        runTrue = False
        if self.estopActive:
            self.almStatusLab.configure(
                text="Estop Button was Pressed", fg_color="red")
        elif self.posOutreach:
            self.almStatusLab.configure(
                text="Position Out of Reach", fg_color="red")
        else:
            self.almStatusLab.configure(text="PROGRAM STOPPED", fg_color="red")

    def executeRow(self):
        selRow = self.tab1.progView.curselection()[0]
        self.tab1.progView.see(selRow + 2)
        command = self.tab1.progView.get(selRow).strip()
        cmdType = command[:6]

        # Dictionary mapping command types to methods
        command_map = {
            "Call P": self.callProgram,
            "Run Gc": self.runGcodeProgram,
            "Return": self.returnProgram,
            "Test L": self.testLimitSwitches,
            "Set En": self.setEncoders,
            "Read E": self.readEncoders,
            "Servo ": self.sendServoCommand,
            "If Inp": self.processIfInput,
            "Read C": self.processReadCom,
            "If Reg": self.processIfRegister,
            "If COM": self.processIfCom,
            "TifOn ": self.processInputOnJump,
            "TifOff": self.processInputOffJump,
            "Jump T": self.processJumpToRow,
            "Out On": lambda cmd: self.processSetOutputOn(cmd, self.ser2),
            "Out Of": lambda cmd: self.processSetOutputOff(cmd, self.ser2),
            "ToutOn": lambda cmd: self.processSetOutputOn(cmd, self.ser),
            "ToutOf": lambda cmd: self.processSetOutputOff(cmd, self.ser),
            "Wait I": lambda cmd: self.processWaitInputOn(cmd, self.ser2),
            "Wait O": lambda cmd: self.processWaitInputOff(cmd, self.ser2),
            "TwaitI": lambda cmd: self.processWaitInputOn(cmd, self.ser),
            "TwaitO": lambda cmd: self.processWaitInputOff(cmd, self.ser),
            "Wait T": self.processWaitTime,
            "Regist": self.processSetRegister,
            "Positi": self.processSetPositionRegister,
            "Calibr": self.processCalibrate,
            "Tool S": self.processToolS,
            "Move J": self.processMoveJ,
            "OFF J": self.processOffJ,
            "Move V": self.handleMoveVCommand,
            "Move P": self.handleMovePCommand,
            "OFF PR": self.handleOffsPRCommand,
            "Move L": self.handleMoveL,
            "Move R": self.handleMoveR,
            "Move A": self.handleMoveA,
            "Move C": self.handleMoveC,
            "Start ": self.startSpline,
            "End Sp": self.endSpline,
            "Cam On": self.cameraOn,
            "Cam Of": self.cameraOff,
            "Vis Fi": self.visionFind,
        }

        # Call the appropriate command function if it exists in the map
        command_func = command_map.get(cmdType)
        if command_func:
            command_func(command)

    def callProgram(self, command):
        if self.moveInProc:
            self.moveInProc = 2
        lastRow = self.tab1.progView.curselection()[0]
        lastProg = self.ProgEntryField.get()

        # Extract the program number
        programIndex = command.find("Program -")
        progNum = command[programIndex + 10:].strip()

        self.ProgEntryField.delete(0, 'end')
        self.ProgEntryField.insert(0, progNum)
        self.callProg(progNum)

        time.sleep(0.4)
        # Reset the selection to the start
        self.tab1.progView.selection_clear(0, END)
        self.tab1.progView.select_set(0)

    def runGcodeProgram(self, command):
        if self.moveInProc:
            self.moveInProc = 2
        lastRow = self.tab1.progView.curselection()[0]
        lastProg = self.ProgEntryField.get()

        # Extract the filename
        programIndex = command.find("Program -")
        filename = command[programIndex + 10:].strip()

        self.manEntryField.delete(0, 'end')
        self.manEntryField.insert(0, filename)

        self.GCplayProg(filename)

        time.sleep(0.4)
        # Reset the selection to the start
        self.tab1.progView.selection_clear(0, END)
        self.tab1.progView.select_set(0)

    def returnProgram(self):
        if self.moveInProc:
            self.moveInProc = 2
        lastRow = lastRow
        lastProg = lastProg

        self.ProgEntryField.delete(0, 'end')
        self.ProgEntryField.insert(0, lastProg)
        self.callProg(lastProg)

        time.sleep(0.4)
        # Re-select the last row
        self.tab1.progView.selection_clear(0, END)
        self.tab1.progView.select_set(lastRow)

    def testLimitSwitches(self):
        if self.moveInProc:
            self.moveInProc = 2
        command = "TL\n"
        self.cmdSentEntryField.delete(0, 'end')
        self.cmdSentEntryField.insert(0, command)
        self.ser.write(command.encode())
        self.ser.flushInput()
        time.sleep(0.05)
        response = str(self.ser.readline().strip(), 'utf-8')
        self.manEntryField.delete(0, 'end')
        self.manEntryField.insert(0, response)

    def setEncoders(self):
        if self.moveInProc:
            self.moveInProc = 2
        command = "SE\n"
        self.cmdSentEntryField.delete(0, 'end')
        self.cmdSentEntryField.insert(0, command)
        self.ser.write(command.encode())
        self.ser.flushInput()
        time.sleep(0.05)
        self.ser.read()  # Clearing response from the encoder setting

    def readEncoders(self):
        if self.moveInProc:
            self.moveInProc = 2
        command = "RE\n"
        self.cmdSentEntryField.delete(0, 'end')
        self.cmdSentEntryField.insert(0, command)
        self.ser.write(command.encode())
        self.ser.flushInput()
        time.sleep(0.05)
        response = str(self.ser.readline().strip(), 'utf-8')
        self.manEntryField.delete(0, 'end')
        self.manEntryField.insert(0, response)

    def sendServoCommand(self, command):
        if self.moveInProc:
            self.moveInProc = 2
        servoIndex = command.find("number ")
        posIndex = command.find("position: ")
        servoNum = command[servoIndex + 7: posIndex - 4].strip()
        servoPos = command[posIndex + 10:].strip()
        command = f"SV{servoNum}P{servoPos}\n"
        self.cmdSentEntryField.delete(0, 'end')
        self.cmdSentEntryField.insert(0, command)
        self.ser2.write(command.encode())
        self.ser2.flushInput()
        time.sleep(0.1)
        self.ser2.read()  # Clearing response from the servo command

    def processIfInput(self, command):
        if self.moveInProc:
            self.moveInProc = 2

        # Parsing command
        args = ("# ", "= ", ": ")
        input_num, val_num, action = [
            command[command.find(arg) + len(arg):].split()[0] for arg in args
        ]

        # Send query command
        query_cmd = f"JFX{input_num}\n"
        self.cmdSentEntryField.delete(0, 'end')
        self.cmdSentEntryField.insert(0, query_cmd)
        self.ser2.write(query_cmd.encode())
        self.ser2.flushInput()
        time.sleep(0.1)

        # Read and evaluate serial response
        response = str(self.ser2.readline().strip(), 'utf-8')
        query = 1 if response == "T" else 0
        if query == val_num:
            if action == "Call":
                # Handle "Call" action
                self.tab1.lastRow = self.tab1.progView.curselection()[0]
                self.tab1.lastProg = self.ProgEntryField.get()
                prog_name = command[command.find("Prog") + 5:] + ".ar"
                self.callProg(prog_name)

                # Reset selection in progView
                index = 0
                self.tab1.progView.selection_clear(0, "end")
                self.tab1.progView.select_set(index)

            elif action == "Jump":
                # Handle "Jump" action directly
                tab_num = command[command.find("Tab") + 4:]
                encoded_tab = ("Tab Number " + tab_num + "\r\n").encode('utf-8')
                index = self.tab1.progView.get(0, "end").index(encoded_tab) - 1
                self.tab1.progView.selection_clear(0, "end")
                self.tab1.progView.select_set(index)

    def processReadCom(self, command):
        # Parsing command arguments
        args = ("# ", "Char: ", ": ")
        com_num, char_num = [
            command[command.find(arg) + len(arg):].split()[0] for arg in args
        ]

        # Attempt to establish serial communication
        try:
            self.ser3 = serial.Serial(f"COM{com_num}", 115200, timeout=10)
        except:
            # Log error if connection fails
            timestamp = datetime.datetime.now().strftime("%B %d %Y - %I:%M%p")
            error_message = f"{timestamp} - UNABLE TO ESTABLISH COMMUNICATIONS WITH SERIAL DEVICE"
            self.tab8.ElogView.insert(END, error_message)
            error_log = self.tab8.ElogView.get(0, END)
            pickle.dump(error_log, open("ErrorLog", "wb"))
            return

        # Read the serial response
        response = (
            str(self.ser3.read(int(char_num)).strip(), 'utf-8')
            if char_num.isdigit()
            else str(self.ser3.readline().strip(), 'utf-8')
        )

        # Update entry fields with the response
        for field in [self.com3outPortEntryField, self.manEntryField]:
            field.delete(0, 'end')
            field.insert(0, response)

    def processIfRegister(self, command):
        if self.moveInProc:
            self.moveInProc = 2

        # Parse command to get input number, value number, and action
        args = ("# ", "= ", ": ")
        input_num, val_num, action = [
            command[command.find(arg) + len(arg):].split()[0] for arg in args
        ]

        # Get register value
        reg_value = int(eval(f"R{input_num}EntryField").get())

        # Check if register value matches
        if reg_value == val_num:
            if action == "Call":
                # Handle "Call" action
                self.tab1.lastRow = self.tab1.progView.curselection()[0]
                self.tab1.lastProg = self.ProgEntryField.get()
                prog_name = command[command.find("Prog") + 5:] + ".ar"
                self.callProg(prog_name)
                
                # Reset selection in progView
                index = 0
                self.tab1.progView.selection_clear(0, "end")
                self.tab1.progView.select_set(index)

            elif action == "Jump":
                # Handle "Jump" action within this method
                tab_num = command[command.find("Tab") + 4:]
                encoded_tab = ("Tab Number " + tab_num + "\r\n").encode('utf-8')
                index = self.tab1.progView.get(0, "end").index(encoded_tab) - 1
                self.tab1.progView.selection_clear(0, "end")
                self.tab1.progView.select_set(index)

    def processIfCom(self, command):
        if self.moveInProc:
            self.moveInProc = 2

        # Parse command to get input number, value number, and action
        args = ("# ", "= ", ": ")
        input_num, val_num, action = [
            command[command.find(arg) + len(arg):].split()[0] for arg in args
        ]

        # Get current COM port value
        cur_com_val = self.com3outPortEntryField.get()

        # Check if COM port value matches
        if cur_com_val == val_num:
            if action == "Call":
                # Handle "Call" action
                self.tab1.lastRow = self.tab1.progView.curselection()[0]
                self.tab1.lastProg = self.ProgEntryField.get()
                prog_name = command[command.find("Prog") + 5:] + ".ar"
                self.callProg(prog_name)
                
                # Reset selection in progView
                index = 0
                self.tab1.progView.selection_clear(0, END)
                self.tab1.progView.select_set(index)
                
            elif action == "Jump":
                # Handle "Jump" action
                tab_num = command[command.find("Tab") + 4:]
                encoded_tab = ("Tab Number " + tab_num + "\r\n").encode('utf-8')
                
                # Find and select the tab by index in progView
                index = self.tab1.progView.get(0, "end").index(encoded_tab) - 1
                self.tab1.progView.selection_clear(0, END)
                self.tab1.progView.select_set(index)

    def processInputOnJump(self, command):
        if self.moveInProc:
            self.moveInProc = 2

        # Parse command to get input number and tab number
        args = ("Input-", "Tab-", "")
        input_num, tab_num = [
            command[command.find(arg) + len(arg):].split()[0] for arg in args
        ]

        # Construct and send jump command
        jump_command = f"JFX{input_num}T{tab_num}\n"
        self.cmdSentEntryField.delete(0, 'end')
        self.cmdSentEntryField.insert(0, jump_command)
        self.ser.write(jump_command.encode())
        self.ser.flushInput()
        time.sleep(0.1)

        # Read serial response
        response = str(self.ser.readline().strip(), 'utf-8')
        
        # If response is "T", proceed to jump to the specified tab
        if response == "T":
            encoded_tab = ("Tab Number " + tab_num + "\r\n").encode('utf-8')
            index = self.tab1.progView.get(0, "end").index(encoded_tab) - 1
            self.tab1.progView.selection_clear(0, END)
            self.tab1.progView.select_set(index)

    def processInputOffJump(self, command):
        if self.moveInProc:
            self.moveInProc = 2

        # Parse command to get input number and tab number
        args = ("Input-", "Tab-", "")
        input_num, tab_num = [
            command[command.find(arg) + len(arg):].split()[0] for arg in args
        ]

        # Construct and send jump command
        jump_command = f"JFX{input_num}T{tab_num}\n"
        self.cmdSentEntryField.delete(0, 'end')
        self.cmdSentEntryField.insert(0, jump_command)
        self.ser.write(jump_command.encode())
        self.ser.flushInput()
        time.sleep(0.1)

        # Read serial response
        response = str(self.ser.readline().strip(), 'utf-8')
        
        # If response is "F", proceed to jump to the specified tab
        if response == "F":
            encoded_tab = ("Tab Number " + tab_num + "\r\n").encode('utf-8')
            index = self.tab1.progView.get(0, "end").index(encoded_tab) - 1
            self.tab1.progView.selection_clear(0, END)
            self.tab1.progView.select_set(index)

    def processJumpToRow(self, command):
        if self.moveInProc:
            self.moveInProc = 2

        # Extract tab number directly
        start_str = "Tab-"
        start_idx = command.find(start_str) + len(start_str)
        tab_num = command[start_idx:]

        # Locate and select the tab in progView
        encoded_tab = ("Tab Number " + tab_num + "\r\n").encode('utf-8')
        index = self.tab1.progView.get(0, "end").index(encoded_tab)
        self.tab1.progView.selection_clear(0, END)
        self.tab1.progView.select_set(index)

    def processSetOutputOn(self, command, ser):
        if self.moveInProc:
            self.moveInProc = 2

        # Extract output number directly
        start_str = "Out On = "
        start_idx = command.find(start_str) + len(start_str)
        output_num = command[start_idx:]

        # Send I/O command
        io_command = f"ONX{output_num}\n"
        self.cmdSentEntryField.delete(0, 'end')
        self.cmdSentEntryField.insert(0, io_command)
        self.ser.write(io_command.encode())
        self.ser.flushInput()
        time.sleep(0.1)
        self.ser.read()

    def processSetOutputOff(self, command, ser):
        if self.moveInProc:
            self.moveInProc = 2

        # Extract output number directly
        start_str = "Out Off = "
        start_idx = command.find(start_str) + len(start_str)
        output_num = command[start_idx:]

        # Send I/O command
        io_command = f"OFX{output_num}\n"
        self.cmdSentEntryField.delete(0, 'end')
        self.cmdSentEntryField.insert(0, io_command)
        self.ser.write(io_command.encode())
        self.ser.flushInput()
        time.sleep(0.1)
        self.ser.read()

    # Handle 'Wait Input ON' command for the given serial connection.
    def processWaitInputOn(self, command, ser):
        if self.moveInProc:
            self.moveInProc = 2

        # Extract input number directly
        start_str = "Wait Input On = "
        start_idx = command.find(start_str) + len(start_str)
        input_num = command[start_idx:]

        # Send wait command
        wait_command = f"WIN{input_num}\n"
        self.cmdSentEntryField.delete(0, 'end')
        self.cmdSentEntryField.insert(0, wait_command)
        self.ser.write(wait_command.encode())
        self.ser.flushInput()
        time.sleep(0.1)
        self.ser.read()

    # Handle 'Wait Input OFF' command for the given serial connection.
    def processWaitInputOff(self, command, ser):
        if self.moveInProc:
            self.moveInProc = 2

        # Extract input number directly
        start_str = "Wait Off Input = "
        start_idx = command.find(start_str) + len(start_str)
        input_num = command[start_idx:]

        # Send wait command
        wait_command = f"WON{input_num}\n"
        self.cmdSentEntryField.delete(0, 'end')
        self.cmdSentEntryField.insert(0, wait_command)
        self.ser.write(wait_command.encode())
        self.ser.flushInput()
        time.sleep(0.1)
        self.ser.read()

    # Handle 'Wait Time' command.
    def processWaitTime(self, command):
        if self.moveInProc:
            self.moveInProc = 2

        # Extract wait time in seconds
        start_str = "Wait Time = "
        start_idx = command.find(start_str) + len(start_str)
        time_seconds = command[start_idx:]

        # Send wait command
        wait_command = f"WTS{time_seconds}\n"
        self.cmdSentEntryField.delete(0, 'end')
        self.cmdSentEntryField.insert(0, wait_command)
        self.ser.write(wait_command.encode())
        self.ser.flushInput()
        time.sleep(0.1)
        self.ser.read()

    # Handles the 'Set Register' command.
    def processSetRegister(self, command):
        if self.moveInProc:
            self.moveInProc = 2

        # Extract register number
        start_str = "Register "
        end_str = " = "
        start_idx = command.find(start_str) + len(start_str)
        end_idx = command.find(end_str, start_idx)
        reg_num = command[start_idx:end_idx]
        reg_entry_field = f"R{reg_num}EntryField"

        # Calculate register value
        reg_eq_index = command.find(" = ")
        test_oper = command[reg_eq_index + 3:reg_eq_index + 5]
        reg_current_val = int(eval(reg_entry_field).get())
        
        if test_oper == "++":
            reg_change_val = int(command[reg_eq_index + 5:])
            reg_new_val = reg_current_val + reg_change_val
        elif test_oper == "--":
            reg_change_val = int(command[reg_eq_index + 5:])
            reg_new_val = reg_current_val - reg_change_val
        else:
            reg_new_val = int(command[reg_eq_index + 3:])

        # Update entry field with the new register value
        entry_field = eval(reg_entry_field)
        entry_field.delete(0, 'end')
        entry_field.insert(0, str(reg_new_val))

    # Handles the 'Set Position Register' command.
    def processSetPositionRegister(self, command):
        if self.moveInProc:
            self.moveInProc = 2

        # Extract position register number and element
        start_str = "Position Register "
        end_str = " Element"
        start_idx = command.find(start_str) + len(start_str)
        end_idx = command.find(end_str, start_idx)
        reg_num = command[start_idx:end_idx]

        element_str = "Element"
        element_end_str = " = "
        element_start_idx = command.find(element_str) + len(element_str)
        element_end_idx = command.find(element_end_str, element_start_idx)
        reg_element = command[element_start_idx:element_end_idx]

        reg_entry_field = f"SP_{reg_num}_E{reg_element}_EntryField"

        # Calculate the new register value
        reg_eq_index = command.find(" = ")
        test_oper = command[reg_eq_index + 3:reg_eq_index + 5]
        reg_current_val = float(eval(reg_entry_field).get())

        if test_oper == "++":
            reg_change_val = float(command[reg_eq_index + 5:])
            reg_new_val = reg_current_val + reg_change_val
        elif test_oper == "--":
            reg_change_val = float(command[reg_eq_index + 5:])
            reg_new_val = reg_current_val - reg_change_val
        else:
            reg_new_val = float(command[reg_eq_index + 3:])

        # Update the entry field with the new position register value
        entry_field = eval(reg_entry_field)
        entry_field.delete(0, 'end')
        entry_field.insert(0, str(reg_new_val))

    # Handles the 'Calibrate' command.
    def processCalibrate(self):
        if self.moveInProc:
            self.moveInProc = 2
        self.calRobotAll()
        if self.calStat == 0:
            self.stopProg()

    # Process the Tool S command, send it to the device, handle errors, and display position data.
    def processToolS(self, command):
        # Set move process state and system status
        if self.moveInProc == 1:
            self.moveInProc = 2
        statusText = "SYSTEM READY"
        self.almStatusLab.configure(text=statusText, style="OK.TLabel")
        self.almStatusLab2.configure(text=statusText, style="OK.TLabel")

        # Extract coordinates for Tool S
        xIndex = command.find(" X ")
        yIndex = command.find(" Y ")
        zIndex = command.find(" Z ")
        rzIndex = command.find(" Rz ")
        ryIndex = command.find(" Ry ")
        rxIndex = command.find(" Rx ")
        xVal = command[xIndex+3:yIndex]
        yVal = command[yIndex+3:zIndex]
        zVal = command[zIndex+3:rzIndex]
        rzVal = command[rzIndex+4:ryIndex]
        ryVal = command[ryIndex+4:rxIndex]
        rxVal = command[rxIndex+4:]

        # Populate entry fields with extracted values
        for field, value in zip(
                [self.TFxEntryField, self.TFyEntryField, self.TFzEntryField, 
                self.TFrzEntryField, self.TFryEntryField, self.TFrxEntryField],
                [self.xVal, self.yVal, self.zVal, self.rzVal, self.ryVal, self.rxVal]):
            field.delete(0, 'end')
            field.insert(0, value)

        # Format and send the command
        formattedCommand = f"TF A{xVal} B{yVal} C{zVal} D{rzVal} E{ryVal} F{rxVal}\n"
        self.cmdSentEntryField.delete(0, 'end')
        self.cmdSentEntryField.insert(0, formattedCommand)
        self.ser.write(formattedCommand.encode())
        self.ser.flushInput()
        time.sleep(0.1)
        self.ser.write(formattedCommand.encode())
        self.ser.flushInput()
        time.sleep(0.1)

        # Read the response
        response = str(self.ser.readline().strip(), 'utf-8')
        if response.startswith('E'):
            # Display error message
            self.errorStatusLabel.configure(text=response, style="Error.TLabel")
            print(f"Error: {response}")
        else:
            # Display position data from the response
            try:
                position_fields = {
                    "X": self.PositionXField, "Y": self.PositionYField, "Z": self.PositionZField,
                    "Rz": self.PositionRzField, "Ry": self.PositionRyField, "Rx": self.PositionRxField
                }
                for key, field in position_fields.items():
                    # Extract value for each position key
                    index = response.find(f"{key}")
                    if index == -1:
                        raise ValueError(f"Label '{key}' not found in response.")
                    start = index + len(key)
                    end = response.find(" ", start)
                    value = response[start:end] if end != -1 else response[start:]
                    
                    # Update the field with the extracted value
                    field.delete(0, 'end')
                    field.insert(0, value)

            except Exception as e:
                errorMsg = f"Failed to display position: {str(e)}"
                self.errorStatusLabel.configure(text=errorMsg, style="Error.TLabel")
                print(f"Error: {errorMsg}")

    def processMoveJ(self, command):
        if self.moveInProc == 0:
            self.moveInProc = 1

        # Extract coordinates and settings specific to Move J
        xIndex = command.find(" X ")
        yIndex = command.find(" Y ")
        zIndex = command.find(" Z ")
        rzIndex = command.find(" Rz ")
        ryIndex = command.find(" Ry ")
        rxIndex = command.find(" Rx ")
        J7Index = command.find(" J7 ")
        J8Index = command.find(" J8 ")
        J9Index = command.find(" J9 ")
        SpeedIndex = command.find(" S")
        ACCspdIndex = command.find(" Ac ")
        DECspdIndex = command.find(" Dc ")
        ACCrampIndex = command.find(" Rm ")
        WristConfIndex = command.find(" $")

        xVal = command[xIndex+3:yIndex]
        yVal = command[yIndex+3:zIndex]
        zVal = command[zIndex+3:rzIndex]
        rzVal = command[rzIndex+4:ryIndex]
        ryVal = command[ryIndex+4:rxIndex]
        rxVal = command[rxIndex+4:J7Index]
        J7Val = command[J7Index+4:J8Index]
        J8Val = command[J8Index+4:J9Index]
        J9Val = command[J9Index+4:SpeedIndex]
        speedPrefix = command[SpeedIndex+1:SpeedIndex+3]
        speed = command[SpeedIndex+4:ACCspdIndex]
        ACCspd = command[ACCspdIndex+4:DECspdIndex]
        DECspd = command[DECspdIndex+4:ACCrampIndex]
        ACCramp = command[ACCrampIndex+4:WristConfIndex]
        WC = command[WristConfIndex+3:]
        LoopMode = (str(self.J1OpenLoopStat.get()) + str(self.J2OpenLoopStat.get()) +
                    str(self.J3OpenLoopStat.get()) + str(self.J4OpenLoopStat.get()) +
                    str(self.J5OpenLoopStat.get()) + str(self.J6OpenLoopStat.get()))

        # Format and send command
        formattedCommand = (f"MJ X{xVal} Y{yVal} Z{zVal} Rz{rzVal} Ry{ryVal} Rx{rxVal} "
                            f"J7{J7Val} J8{J8Val} J9{J9Val} {speedPrefix}{speed} "
                            f"Ac{ACCspd} Dc{DECspd} Rm{ACCramp} W{WC} Lm{LoopMode}\n")
        self.cmdSentEntryField.delete(0, 'end')
        self.cmdSentEntryField.insert(0, formattedCommand)
        self.ser.write(formattedCommand.encode())
        self.ser.flushInput()
        time.sleep(0.1)
        self.ser.write(formattedCommand.encode())
        self.ser.flushInput()
        time.sleep(0.1)

        # Read and handle response
        response = str(self.ser.readline().strip(), 'utf-8')
        if response.startswith('E'):
            # Handle errors by displaying the error message
            self.errorStatusLabel.configure(text=response, style="Error.TLabel")
            print(f"Error: {response}")
        else:
            # Display position data based on response received from the device
            try:
                def extract_value(response, label):
                    index = response.find(f"{label}")
                    if index == -1:
                        raise ValueError(f"Label '{label}' not found in response.")
                    start = index + len(label)
                    end = response.find(" ", start)
                    return response[start:end] if end != -1 else response[start:]

                position_values = {
                    "X": extract_value(response, "X"),
                    "Y": extract_value(response, "Y"),
                    "Z": extract_value(response, "Z"),
                    "Rz": extract_value(response, "Rz"),
                    "Ry": extract_value(response, "Ry"),
                    "Rx": extract_value(response, "Rx"),
                }

                # Populate the position fields
                for key, field in zip(
                        ["X", "Y", "Z", "Rz", "Ry", "Rx"],
                        [self.PositionXField, self.PositionYField, self.PositionZField, 
                        self.PositionRzField, self.PositionRyField, self.PositionRxField]):
                    field.delete(0, 'end')
                    field.insert(0, position_values[key])

            except Exception as e:
                errorMsg = f"Failed to display position: {str(e)}"
                self.errorStatusLabel.configure(text=errorMsg, style="Error.TLabel")
                print(f"Error: {errorMsg}")

    def processOffJ(self, command):
        if self.moveInProc == 0:
            self.moveInProc = 1

        # Extract SP and command data for Off J
        SPnewIndex = command.find("[ PR: ")
        SPendIndex = command.find(" ] [")
        SP = command[SPnewIndex + 6:SPendIndex]

        # Helper function to retrieve offsets
        def get_offset(fieldName):
            field = getattr(fieldName, None)
            if field:
                return field.get()
            else:
                raise ValueError(f"Field '{fieldName}' does not exist.")

        # Get current offsets for Off J
        cx = get_offset(f"SP_{SP}_E1_EntryField")
        cy = get_offset(f"SP_{SP}_E2_EntryField")
        cz = get_offset(f"SP_{SP}_E3_EntryField")
        crz = get_offset(f"SP_{SP}_E4_EntryField")
        cry = get_offset(f"SP_{SP}_E5_EntryField")
        crx = get_offset(f"SP_{SP}_E6_EntryField")

        # Extract movement data
        def extract_move_j_data(command):
            xIndex = command.find(" X ")
            yIndex = command.find(" Y ")
            zIndex = command.find(" Z ")
            rzIndex = command.find(" Rz ")
            ryIndex = command.find(" Ry ")
            rxIndex = command.find(" Rx ")
            J7Index = command.find(" J7 ")
            J8Index = command.find(" J8 ")
            J9Index = command.find(" J9 ")
            SpeedIndex = command.find(" S")
            ACCspdIndex = command.find(" Ac ")
            DECspdIndex = command.find(" Dc ")
            ACCrampIndex = command.find(" Rm ")
            WristConfIndex = command.find(" $")

            xVal = command[xIndex+3:yIndex]
            yVal = command[yIndex+3:zIndex]
            zVal = command[zIndex+3:rzIndex]
            rzVal = command[rzIndex+4:ryIndex]
            ryVal = command[ryIndex+4:rxIndex]
            rxVal = command[rxIndex+4:J7Index]
            J7Val = command[J7Index+4:J8Index]
            J8Val = command[J8Index+4:J9Index]
            J9Val = command[J9Index+4:SpeedIndex]
            speedPrefix = command[SpeedIndex+1:SpeedIndex+3]
            speed = command[SpeedIndex+4:ACCspdIndex]
            ACCspd = command[ACCspdIndex+4:DECspdIndex]
            DECspd = command[DECspdIndex+4:ACCrampIndex]
            ACCramp = command[ACCrampIndex+4:WristConfIndex]
            WC = command[WristConfIndex+3:]
            LoopMode = (str(self.J1OpenLoopStat.get()) + str(self.J2OpenLoopStat.get()) +
                        str(self.J3OpenLoopStat.get()) + str(self.J4OpenLoopStat.get()) +
                        str(self.J5OpenLoopStat.get()) + str(self.J6OpenLoopStat.get()))

            return xVal, yVal, zVal, rzVal, ryVal, rxVal, J7Val, J8Val, J9Val, speedPrefix, speed, ACCspd, DECspd, ACCramp, WC, LoopMode

        (xVal, yVal, zVal, rzVal, ryVal, rxVal, J7Val, J8Val, J9Val, speedPrefix, speed,
        ACCspd, DECspd, ACCramp, WC, LoopMode) = extract_move_j_data(command)

        # Adjust for offsets
        xVal = str(float(cx) + float(xVal))
        yVal = str(float(cy) + float(yVal))
        zVal = str(float(cz) + float(zVal))
        rzVal = str(float(crz) + float(rzVal))
        ryVal = str(float(cry) + float(ryVal))
        rxVal = str(float(crx) + float(rxVal))

        # Format and send command
        formattedCommand = (f"MJ X{xVal} Y{yVal} Z{zVal} Rz{rzVal} Ry{ryVal} Rx{rxVal} "
                            f"J7{J7Val} J8{J8Val} J9{J9Val} {speedPrefix}{speed} "
                            f"Ac{ACCspd} Dc{DECspd} Rm{ACCramp} W{WC} Lm{LoopMode}\n")
        self.cmdSentEntryField.delete(0, 'end')
        self.cmdSentEntryField.insert(0, formattedCommand)
        self.ser.write(formattedCommand.encode())
        self.ser.flushInput()
        time.sleep(0.1)
        self.ser.write(formattedCommand.encode())
        self.ser.flushInput()
        time.sleep(0.1)

        # Read and handle response
        response = str(self.ser.readline().strip(), 'utf-8')
        if response.startswith('E'):
            # Handle errors by displaying the error message
            self.errorStatusLabel.configure(text=response, style="Error.TLabel")
            print(f"Error: {response}")
        else:
            # Extract position values from response and update fields
            try:
                def extract_value(response, label):
                    index = response.find(f"{label}")
                    if index == -1:
                        raise ValueError(f"Label '{label}' not found in response.")
                    start = index + len(label)
                    end = response.find(" ", start)
                    return response[start:end] if end != -1 else response[start:]

                position_values = {
                    "X": extract_value(response, "X"),
                    "Y": extract_value(response, "Y"),
                    "Z": extract_value(response, "Z"),
                    "Rz": extract_value(response, "Rz"),
                    "Ry": extract_value(response, "Ry"),
                    "Rx": extract_value(response, "Rx"),
                }

                # Populate the position fields
                for key, field in zip(
                        ["X", "Y", "Z", "Rz", "Ry", "Rx"],
                        [self.PositionXField, self.PositionYField, self.PositionZField, 
                        self.PositionRzField, self.PositionRyField, self.PositionRxField]):
                    field.delete(0, 'end')
                    field.insert(0, position_values[key])

            except Exception as e:
                errorMsg = f"Failed to display position: {str(e)}"
                self.errorStatusLabel.configure(text=errorMsg, style="Error.TLabel")
                print(f"Error: {errorMsg}")

    # Process the Move V command, send it to the device, handle errors, and display position.
    def handleMoveVCommand(self, command):

        # Ensure movement is in progress
        if self.moveInProc == 0:
            self.moveInProc = 1

        # Extract SP and index positions in the command string
        SPnewIndex = command.find("[ PR: ")
        SPendIndex = command.find(" ] [")
        xIndex, yIndex, zIndex = command.find(" X "), command.find(" Y "), command.find(" Z ")
        rzIndex, ryIndex, rxIndex = command.find(" Rz "), command.find(" Ry "), command.find(" Rx ")
        J7Index, J8Index, J9Index = command.find(" J7 "), command.find(" J8 "), command.find(" J9 ")
        SpeedIndex, ACCspdIndex = command.find(" S"), command.find(" Ac ")
        DECspdIndex, ACCrampIndex = command.find(" Dc "), command.find(" Rm ")
        WristConfIndex = command.find(" $")

        # Define an internal helper to retrieve the offset
        def get_offset(field_name):
            field = getattr(field_name, None)
            if field:
                return field.get()
            else:
                raise ValueError(f"Field '{field_name}' does not exist.")

        # Extract parameters and offsets
        SP = command[SPnewIndex + 6:SPendIndex]
        cx, cy, cz = get_offset(f"SP_{SP}_E1_EntryField"), get_offset(f"SP_{SP}_E2_EntryField"), get_offset(f"SP_{SP}_E3_EntryField")
        crz, cry, crx = get_offset(f"SP_{SP}_E4_EntryField"), get_offset(f"SP_{SP}_E5_EntryField"), get_offset(f"SP_{SP}_E6_EntryField")

        xVal = str(float(cx) + float(self.VisRetXrobEntryField.get()))
        yVal = str(float(cy) + float(self.VisRetYrobEntryField.get()))
        zVal = str(float(cz) + float(command[zIndex + 3:rzIndex]))
        rzVal = str(float(crz) + float(command[rzIndex + 4:ryIndex]))
        ryVal = str(float(cry) + float(command[ryIndex + 4:rxIndex]))
        rxVal = str(float(crx) + float(command[rxIndex + 4:J7Index]))
        J7Val, J8Val, J9Val = command[J7Index + 4:J8Index], command[J8Index + 4:J9Index], command[J9Index + 4:SpeedIndex]
        speedPrefix, Speed = command[SpeedIndex + 1:SpeedIndex + 3], command[SpeedIndex + 4:ACCspdIndex]
        ACCspd, DECspd = command[ACCspdIndex + 4:DECspdIndex], command[DECspdIndex + 4:ACCrampIndex]
        ACCramp, WC = command[ACCrampIndex + 4:WristConfIndex], command[WristConfIndex + 3:]
        visRot = self.VisRetAngleEntryField.get()
        LoopMode = ''.join(str(getattr(f'J{i}OpenLoopStat').get()) for i in range(1, 7))

        # Format command
        formatted_command = (
            f"MV X{xVal} Y{yVal} Z{zVal} Rz{rzVal} Ry{ryVal} Rx{rxVal} "
            f"J7{J7Val} J8{J8Val} J9{J9Val} {speedPrefix}{Speed} "
            f"Ac{ACCspd} Dc{DECspd} Rm{ACCramp} W{WC} Vr{visRot} Lm{LoopMode}\n"
        )

        # Send command and handle response
        self.cmdSentEntryField.delete(0, 'end')
        self.cmdSentEntryField.insert(0, formatted_command)
        self.ser.write(formatted_command.encode())
        self.ser.flushInput()
        time.sleep(0.1)

        # Read and handle response
        response = str(self.ser.readline().strip(), 'utf-8')
        if response.startswith('E'):
            # Handle errors
            error_msg = f"Error: {response}"
            print(error_msg)
            self.errorStatusLabel.configure(text=response, style="Error.TLabel")
        else:
            # Display position
            try:
                # Extracts a numeric value from the response string given a label prefix.
                def extract_value(response, label):
                    index = response.find(f"{label}")
                    if index == -1:
                        raise ValueError(f"Label '{label}' not found in response.")
                    start = index + len(label)
                    end = response.find(" ", start)
                    return response[start:end] if end != -1 else response[start:]

                position_fields = {
                    "X": self.PositionXField, "Y": self.PositionYField, "Z": self.PositionZField,
                    "Rz": self.PositionRzField, "Ry": self.PositionRyField, "Rx": self.PositionRxField
                }
                for label, field in position_fields.items():
                    field.delete(0, 'end')
                    field.insert(0, extract_value(response, label))

            except Exception as e:
                error_msg = f"Failed to display position: {str(e)}"
                print(error_msg)
                self.errorStatusLabel.configure(text=error_msg, style="Error.TLabel")

    # Process the MoveP command, send it to the device, handle errors, and display position data.
    def handleMovePCommand(self, command):

        # Begin processing MoveP command
        if self.moveInProc == 0:
            self.moveInProc = 1

        # Parse command components
        SPnewIndex = command.find("[ PR: ")
        SPendIndex = command.find(" ] [")
        J7Index = command.find(" J7 ")
        J8Index = command.find(" J8 ")
        J9Index = command.find(" J9 ")
        SpeedIndex = command.find(" S")
        ACCspdIndex = command.find(" Ac ")
        DECspdIndex = command.find(" Dc ")
        ACCrampIndex = command.find(" Rm ")
        WristConfIndex = command.find(" $")

        SP = str(command[SPnewIndex + 6:SPendIndex])
        cx = eval(f"SP_{SP}_E1_EntryField").get()
        cy = eval(f"SP_{SP}_E2_EntryField").get()
        cz = eval(f"SP_{SP}_E3_EntryField").get()
        crz = eval(f"SP_{SP}_E4_EntryField").get()
        cry = eval(f"SP_{SP}_E5_EntryField").get()
        crx = eval(f"SP_{SP}_E6_EntryField").get()

        xVal = str(float(cx))
        yVal = str(float(cy))
        zVal = str(float(cz))
        rzVal = str(float(crz))
        ryVal = str(float(cry))
        rxVal = str(float(crx))

        J7Val = command[J7Index + 4:J8Index]
        J8Val = command[J8Index + 4:J9Index]
        J9Val = command[J9Index + 4:SpeedIndex]

        speedPrefix = command[SpeedIndex + 1:SpeedIndex + 3]
        Speed = command[SpeedIndex + 4:ACCspdIndex]
        ACCspd = command[ACCspdIndex + 4:DECspdIndex]
        DECspd = command[DECspdIndex + 4:ACCrampIndex]
        ACCramp = command[ACCrampIndex + 4:WristConfIndex]
        WC = command[WristConfIndex + 3:]

        LoopMode = ''.join(
            str(getattr(f'J{i}OpenLoopStat').get()) for i in range(1, 7))

        # Construct and send the final command
        final_command = f"MJX{xVal}Y{yVal}Z{zVal}Rz{rzVal}Ry{ryVal}Rx{rxVal}J7{J7Val}J8{J8Val}J9{J9Val}{speedPrefix}{Speed}Ac{ACCspd}Dc{DECspd}Rm{ACCramp}W{WC}Lm{LoopMode}\n"
        
        # Send the command to the device
        self.cmdSentEntryField.delete(0, 'end')
        self.cmdSentEntryField.insert(0, final_command)
        self.ser.write(final_command.encode())
        self.ser.flushInput()
        time.sleep(0.1)
        
        # Read the response
        response = str(self.ser.readline().strip(), 'utf-8')
        
        # Check for errors in the response
        if response.startswith('E'):
            print(f"Error: {response}")  # Log the error for debugging
            self.errorStatusLabel.configure(text=response, style="Error.TLabel")  # Display error in UI
            return  # Exit if there's an error

        # Process and display position data if no errors
        position_fields = {
            "X": self.PositionXField, "Y": self.PositionYField, "Z": self.PositionZField,
            "Rz": self.PositionRzField, "Ry": self.PositionRyField, "Rx": self.PositionRxField
        }
        
        try:
            for key, field in position_fields.items():
                # Extract value for each position key and update the respective UI field
                index = response.find(f"{key}")
                if index == -1:
                    raise ValueError(f"Label '{key}' not found in response.")
                start = index + len(key)
                end = response.find(" ", start)
                value = response[start:end] if end != -1 else response[start:]
                field.delete(0, 'end')
                field.insert(0, value)
                
        except Exception as e:
            # Handle any issues during the display update
            errorMsg = f"Failed to display position: {str(e)}"
            print(f"Error: {errorMsg}")
            self.errorStatusLabel.configure(text=errorMsg, style="Error.TLabel")

    # Process the OffsPR command, construct the command string, send it to the device, and handle response errors.
    def handleOffsPRCommand(self, command):
        
        # Set move process state
        if self.moveInProc == 0:
            self.moveInProc = 1

        # Extract position and configuration data from the command
        SPnewIndex = command.find("[ PR: ")
        SPendIndex = command.find(" ] offs")
        SP2newIndex = command.find("[ *PR: ")
        SP2endIndex = command.find(" ]  [")
        J7Index = command.find(" J7 ")
        J8Index = command.find(" J8 ")
        J9Index = command.find(" J9 ")
        SpeedIndex = command.find(" S")
        ACCspdIndex = command.find(" Ac ")
        DECspdIndex = command.find(" Dc ")
        ACCrampIndex = command.find(" Rm ")
        WristConfIndex = command.find(" $")

        # Calculate positional offsets
        SP = str(command[SPnewIndex + 6:SPendIndex])
        SP2 = str(command[SP2newIndex + 7:SP2endIndex])

        xVal = str(float(eval(f"SP_{SP}_E1_EntryField").get()) + float(eval(f"SP_{SP2}_E1_EntryField").get()))
        yVal = str(float(eval(f"SP_{SP}_E2_EntryField").get()) + float(eval(f"SP_{SP2}_E2_EntryField").get()))
        zVal = str(float(eval(f"SP_{SP}_E3_EntryField").get()) + float(eval(f"SP_{SP2}_E3_EntryField").get()))
        rzVal = str(float(eval(f"SP_{SP}_E4_EntryField").get()) + float(eval(f"SP_{SP2}_E4_EntryField").get()))
        ryVal = str(float(eval(f"SP_{SP}_E5_EntryField").get()) + float(eval(f"SP_{SP2}_E5_EntryField").get()))
        rxVal = str(float(eval(f"SP_{SP}_E6_EntryField").get()) + float(eval(f"SP_{SP2}_E6_EntryField").get()))

        # Extract joint and configuration parameters
        J7Val = command[J7Index + 4:J8Index]
        J8Val = command[J8Index + 4:J9Index]
        J9Val = command[J9Index + 4:SpeedIndex]
        speedPrefix = command[SpeedIndex + 1:SpeedIndex + 3]
        Speed = command[SpeedIndex + 4:ACCspdIndex]
        ACCspd = command[ACCspdIndex + 4:DECspdIndex]
        DECspd = command[DECspdIndex + 4:ACCrampIndex]
        ACCramp = command[ACCrampIndex + 4:WristConfIndex]
        WC = command[WristConfIndex + 3:]

        # Loop mode configuration
        LoopMode = ''.join(str(getattr(f'J{i}OpenLoopStat').get()) for i in range(1, 7))

        # Construct the full command
        full_command = f"MJX{xVal}Y{yVal}Z{zVal}Rz{rzVal}Ry{ryVal}Rx{rxVal}J7{J7Val}J8{J8Val}J9{J9Val}{speedPrefix}{Speed}Ac{ACCspd}Dc{DECspd}Rm{ACCramp}W{WC}Lm{LoopMode}\n"

        # Send command
        self.cmdSentEntryField.delete(0, 'end')
        self.cmdSentEntryField.insert(0, full_command)
        self.ser.write(full_command.encode())
        self.ser.flushInput()
        time.sleep(0.1)

        # Read the response
        response = str(self.ser.readline().strip(), 'utf-8')

        # Error handling
        if response.startswith('E'):
            print(f"Error: {response}")
            self.errorStatusLabel.configure(text=response, style="Error.TLabel")
            return

        # Update position fields if no error
        position_fields = {
            "X": self.PositionXField, "Y": self.PositionYField, "Z": self.PositionZField,
            "Rz": self.PositionRzField, "Ry": self.PositionRyField, "Rx": self.PositionRxField
        }
        
        try:
            for key, field in position_fields.items():
                # Extract value for each position key
                index = response.find(f"{key}")
                if index == -1:
                    raise ValueError(f"Label '{key}' not found in response.")
                start = index + len(key)
                end = response.find(" ", start)
                value = response[start:end] if end != -1 else response[start:]

                # Update field with extracted value
                field.delete(0, 'end')
                field.insert(0, value)

        except Exception as e:
            errorMsg = f"Failed to display position: {str(e)}"
            print(f"Error: {errorMsg}")
            self.errorStatusLabel.configure(text=errorMsg, style="Error.TLabel")

    def handleMoveL(self, command):
        # Check and start move if not already in process
        if self.moveInProc == 0:
            self.moveInProc = 1

        # Extract move values from command
        def extractMoveSegment(start, end):
            return command[start:end].strip()

        xIndex = command.find(" X ")
        yIndex = command.find(" Y ")
        zIndex = command.find(" Z ")
        rzIndex = command.find(" Rz ")
        ryIndex = command.find(" Ry ")
        rxIndex = command.find(" Rx ")
        J7Index = command.find(" J7 ")
        J8Index = command.find(" J8 ")
        J9Index = command.find(" J9 ")
        SpeedIndex = command.find(" S")
        ACCspdIndex = command.find(" Ac ")
        DECspdIndex = command.find(" Dc ")
        ACCrampIndex = command.find(" Rm ")
        RoundingIndex = command.find(" Rnd ")
        WristConfIndex = command.find(" $")

        xVal = extractMoveSegment(xIndex + 3, yIndex)
        yVal = extractMoveSegment(yIndex + 3, zIndex)
        zVal = extractMoveSegment(zIndex + 3, rzIndex)
        rzVal = extractMoveSegment(rzIndex + 4, ryIndex)
        ryVal = extractMoveSegment(ryIndex + 4, rxIndex)
        rxVal = extractMoveSegment(rxIndex + 4, J7Index)
        J7Val = extractMoveSegment(J7Index + 4, J8Index)
        J8Val = extractMoveSegment(J8Index + 4, J9Index)
        J9Val = extractMoveSegment(J9Index + 4, SpeedIndex)
        Speed = extractMoveSegment(SpeedIndex + 4, ACCspdIndex)
        ACCspd = extractMoveSegment(ACCspdIndex + 4, DECspdIndex)
        DECspd = extractMoveSegment(DECspdIndex + 4, ACCrampIndex)
        ACCramp = extractMoveSegment(ACCrampIndex + 4, RoundingIndex)
        Rounding = extractMoveSegment(RoundingIndex + 5, WristConfIndex)
        WC = command[WristConfIndex + 3:].strip()

        # Adjust rzVal if necessary
        if np.sign(float(rzVal)) != np.sign(float(self.RzcurPos)):
            rzVal = str(float(rzVal) * -1)

        # Retrieve loop mode and disable wrist rotation flag
        LoopMode = ''.join(str(getattr(f'J{i}OpenLoopStat.get()')) for i in range(1, 7))
        DisWrist = str(self.DisableWristRot.get())

        # Construct and send the command
        full_command = (
            f"MLX{xVal}Y{yVal}Z{zVal}Rz{rzVal}Ry{ryVal}Rx{rxVal}J7{J7Val}J8{J8Val}J9{J9Val}"
            f"S{Speed}Ac{ACCspd}Dc{DECspd}Rm{ACCramp}Rnd{Rounding}W{WC}Lm{LoopMode}Q{DisWrist}\n"
        )
        
        # Send the command and handle response
        start = time.time()
        self.cmdSentEntryField.delete(0, 'end')
        self.cmdSentEntryField.insert(0, full_command)

        self.ser.write(full_command.encode())
        self.ser.flushInput()
        time.sleep(0.1)
        response = str(self.ser.readline().strip(), 'utf-8')

        # Optional timing display
        end = time.time()
        # manEntryField.delete(0, 'end')
        # manEntryField.insert(0, end - start)

        # Handle the response
        if response.startswith('E'):
            # Display error message
            print(f"Error: {response}")
            self.errorStatusLabel.configure(text=response, style="Error.TLabel")
        else:
            # Display position data with inline extraction function
            def extractValue(response, label):
                index = response.find(f"{label}")
                if index == -1:
                    raise ValueError(f"Label '{label}' not found in response.")
                start = index + len(label)
                end = response.find(" ", start)
                return response[start:end] if end != -1 else response[start:]

            try:
                self.PositionXField.delete(0, 'end')
                self.PositionXField.insert(0, extractValue(response, "X"))
                self.PositionYField.delete(0, 'end')
                self.PositionYField.insert(0, extractValue(response, "Y"))
                self.PositionZField.delete(0, 'end')
                self.PositionZField.insert(0, extractValue(response, "Z"))
                self.PositionRzField.delete(0, 'end')
                self.PositionRzField.insert(0, extractValue(response, "Rz"))
                self.PositionRyField.delete(0, 'end')
                self.PositionRyField.insert(0, extractValue(response, "Ry"))
                self.PositionRxField.delete(0, 'end')
                self.PositionRxField.insert(0, extractValue(response, "Rx"))
            except Exception as e:
                # Error handling if display fails
                print(f"Failed to display position: {str(e)}")
                self.errorStatusLabel.configure(text=f"Display Error: {str(e)}", style="Error.TLabel")

    def handleMoveR(self, command):
        # Start move if not already in process
        if self.moveInProc == 0:
            self.moveInProc = 1

        # Extract move values from command
        def extractMoveSegment(start, end):
            return command[start:end].strip()

        J1Index = command.find(" J1 ")
        J2Index = command.find(" J2 ")
        J3Index = command.find(" J3 ")
        J4Index = command.find(" J4 ")
        J5Index = command.find(" J5 ")
        J6Index = command.find(" J6 ")
        J7Index = command.find(" J7 ")
        J8Index = command.find(" J8 ")
        J9Index = command.find(" J9 ")
        SpeedIndex = command.find(" S")
        ACCspdIndex = command.find(" Ac ")
        DECspdIndex = command.find(" Dc ")
        ACCrampIndex = command.find(" Rm ")
        WristConfIndex = command.find(" $")

        J1Val = extractMoveSegment(J1Index + 4, J2Index)
        J2Val = extractMoveSegment(J2Index + 4, J3Index)
        J3Val = extractMoveSegment(J3Index + 4, J4Index)
        J4Val = extractMoveSegment(J4Index + 4, J5Index)
        J5Val = extractMoveSegment(J5Index + 4, J6Index)
        J6Val = extractMoveSegment(J6Index + 4, J7Index)
        J7Val = extractMoveSegment(J7Index + 4, J8Index)
        J8Val = extractMoveSegment(J8Index + 4, J9Index)
        J9Val = extractMoveSegment(J9Index + 4, SpeedIndex)
        Speed = extractMoveSegment(SpeedIndex + 4, ACCspdIndex)
        ACCspd = extractMoveSegment(ACCspdIndex + 4, DECspdIndex)
        DECspd = extractMoveSegment(DECspdIndex + 4, ACCrampIndex)
        ACCramp = extractMoveSegment(ACCrampIndex + 4, WristConfIndex)
        WC = command[WristConfIndex + 3:].strip()

        # Retrieve loop mode
        LoopMode = ''.join(str(getattr(f'J{i}OpenLoopStat.get()')) for i in range(1, 7))

        # Construct command
        full_command = (
            f"RJ A{J1Val}B{J2Val}C{J3Val}D{J4Val}E{J5Val}F{J6Val}J7{J7Val}J8{J8Val}J9{J9Val}"
            f"S{Speed}Ac{ACCspd}Dc{DECspd}Rm{ACCramp}W{WC}Lm{LoopMode}\n"
        )

        # Send command and handle response
        start = time.time()
        self.cmdSentEntryField.delete(0, 'end')
        self.cmdSentEntryField.insert(0, full_command)

        self.ser.write(full_command.encode())
        self.ser.flushInput()
        time.sleep(0.1)
        response = str(self.ser.readline().strip(), 'utf-8')

        # Optional timing display
        end = time.time()
        # manEntryField.delete(0, 'end')
        # manEntryField.insert(0, end - start)

        # Handle response
        if response.startswith('E'):
            # Error handling: Display error message
            print(f"Error: {response}")
            self.errorStatusLabel.configure(text=response, style="Error.TLabel")
        else:
            # Display position data with inline extraction function
            def extractValue(response, label):
                index = response.find(f"{label}")
                if index == -1:
                    raise ValueError(f"Label '{label}' not found in response.")
                start = index + len(label)
                end = response.find(" ", start)
                return response[start:end] if end != -1 else response[start:]

            try:
                self.PositionXField.delete(0, 'end')
                self.PositionXField.insert(0, extractValue(response, "X"))
                self.PositionYField.delete(0, 'end')
                self.PositionYField.insert(0, extractValue(response, "Y"))
                self.PositionZField.delete(0, 'end')
                self.PositionZField.insert(0, extractValue(response, "Z"))
                self.PositionRzField.delete(0, 'end')
                self.PositionRzField.insert(0, extractValue(response, "Rz"))
                self.PositionRyField.delete(0, 'end')
                self.PositionRyField.insert(0, extractValue(response, "Ry"))
                self.PositionRxField.delete(0, 'end')
                self.PositionRxField.insert(0, extractValue(response, "Rx"))
            except Exception as e:
                # Error handling if display fails
                print(f"Failed to display position: {str(e)}")
                self.errorStatusLabel.configure(text=f"Display Error: {str(e)}", style="Error.TLabel")

    def handleMoveA(self, command):
        # Start move if not already in process
        if self.moveInProc == 0:
            self.moveInProc = 1

        # Check command validity
        if command.startswith("Move A End"):
            self.almStatusLab.configure(text="Move A must start with a Mid followed by End", style="Alarm.TLabel")
            self.almStatusLab2.configure(text="Move A must start with a Mid followed by End", style="Alarm.TLabel")
            return

        # Extract move values from command
        def extractMoveSegment(start, end):
            return command[start:end].strip()

        xIndex = command.find(" X ")
        yIndex = command.find(" Y ")
        zIndex = command.find(" Z ")
        rzIndex = command.find(" Rz ")
        ryIndex = command.find(" Ry ")
        rxIndex = command.find(" Rx ")
        J7Index = command.find(" J7 ")
        J8Index = command.find(" J8 ")
        J9Index = command.find(" J9 ")
        SpeedIndex = command.find(" S")
        ACCspdIndex = command.find(" Ac ")
        DECspdIndex = command.find(" Dc ")
        ACCrampIndex = command.find(" Rm ")
        RoundingIndex = command.find(" Rnd ")
        WristConfIndex = command.find(" $")

        xVal = extractMoveSegment(xIndex + 3, yIndex)
        yVal = extractMoveSegment(yIndex + 3, zIndex)
        zVal = extractMoveSegment(zIndex + 3, rzIndex)
        rzVal = extractMoveSegment(rzIndex + 4, ryIndex)
        ryVal = extractMoveSegment(ryIndex + 4, rxIndex)
        rxVal = extractMoveSegment(rxIndex + 4, J7Index)
        J7Val = extractMoveSegment(J7Index + 4, J8Index)
        J8Val = extractMoveSegment(J8Index + 4, J9Index)
        J9Val = extractMoveSegment(J9Index + 4, SpeedIndex)
        Speed = extractMoveSegment(SpeedIndex + 4, ACCspdIndex)
        ACCspd = extractMoveSegment(ACCspdIndex + 4, DECspdIndex)
        DECspd = extractMoveSegment(DECspdIndex + 4, ACCrampIndex)
        ACCramp = extractMoveSegment(ACCrampIndex + 4, RoundingIndex)
        Rounding = extractMoveSegment(RoundingIndex + 5, WristConfIndex)
        WC = command[WristConfIndex + 3:].strip()

        # Retrieve end position values
        curRow = self.tab1.progView.curselection()[0]
        selRow = curRow
        last = self.tab1.progView.index('end')

        # Set color for selected rows
        for row in range(0, selRow):
            self.tab1.progView.itemconfig(row, {'fg': 'dodger blue'})
        self.tab1.progView.itemconfig(selRow, {'fg': 'blue2'})
        for row in range(selRow + 1, last):
            self.tab1.progView.itemconfig(row, {'fg': 'black'})

        self.tab1.progView.selection_clear(0, END)
        selRow += 1
        self.tab1.progView.select_set(selRow)
        curRow += 1
        selRow = self.tab1.progView.curselection()[0]
        self.tab1.progView.see(selRow + 2)

        data = list(map(int, self.tab1.progView.curselection()))
        end_command = self.tab1.progView.get(data[0]).decode()

        Xend, Yend, Zend = end_command[:3]  # Extracted Xend, Yend, Zend from end position command

        # Retrieve loop mode
        LoopMode = ''.join(str(getattr(f'J{i}OpenLoopStat.get()')) for i in range(1, 7))

        # Construct command
        full_command = (
            f"MAX{xVal}Y{yVal}Z{zVal}Rz{rzVal}Ry{ryVal}Rx{rxVal}Ex{Xend}Ey{Yend}Ez{Zend}Tr{J7Val}"
            f"S{Speed}Ac{ACCspd}Dc{DECspd}Rm{ACCramp}W{WC}Lm{LoopMode}\n"
        )

        # Send command and handle response
        start = time.time()
        self.cmdSentEntryField.delete(0, 'end')
        self.cmdSentEntryField.insert(0, full_command)

        self.ser.write(full_command.encode())
        self.ser.flushInput()
        time.sleep(0.1)
        response = str(self.ser.readline().strip(), 'utf-8')

        # Optional timing display
        end = time.time()
        # manEntryField.delete(0, 'end')
        # manEntryField.insert(0, end - start)

        # Handle response
        if response.startswith('E'):
            # Error handling: Display error message
            print(f"Error: {response}")
            self.errorStatusLabel.configure(text=response, style="Error.TLabel")
        else:
            # Display position data with inline extraction function
            def extractValue(response, label):
                index = response.find(f"{label}")
                if index == -1:
                    raise ValueError(f"Label '{label}' not found in response.")
                start = index + len(label)
                end = response.find(" ", start)
                return response[start:end] if end != -1 else response[start:]

            try:
                self.PositionXField.delete(0, 'end')
                self.PositionXField.insert(0, extractValue(response, "X"))
                self.PositionYField.delete(0, 'end')
                self.PositionYField.insert(0, extractValue(response, "Y"))
                self.PositionZField.delete(0, 'end')
                self.PositionZField.insert(0, extractValue(response, "Z"))
                self.PositionRzField.delete(0, 'end')
                self.PositionRzField.insert(0, extractValue(response, "Rz"))
                self.PositionRyField.delete(0, 'end')
                self.PositionRyField.insert(0, extractValue(response, "Ry"))
                self.PositionRxField.delete(0, 'end')
                self.PositionRxField.insert(0, extractValue(response, "Rx"))
            except Exception as e:
                # Error handling if display fails
                print(f"Failed to display position: {str(e)}")
                self.errorStatusLabel.configure(text=f"Display Error: {str(e)}", style="Error.TLabel")

    def handleMoveC(self, command):
        if self.moveInProc == 0:
            self.moveInProc = 1

        # Check command format
        subCmd = command[:10]
        if subCmd in ["Move C Sta", "Move C Pla"]:
            # Inline showError logic
            message = "Move C must start with a Center followed by Start & Plane"
            self.almStatusLab.configure(text=message, style="Alarm.TLabel")
            self.almStatusLab2.configure(text=message, style="Alarm.TLabel")
            return

        # Inline extractMoveCValues logic to extract command values
        indices = {
            "X": command.find(" X "),
            "Y": command.find(" Y "),
            "Z": command.find(" Z "),
            "Rz": command.find(" Rz "),
            "Ry": command.find(" Ry "),
            "Rx": command.find(" Rx "),
            "Tr": command.find(" Tr "),
            "S": command.find(" S"),
            "Acc": command.find(" Ac "),
            "Dec": command.find(" Dc "),
            "Rm": command.find(" Rm "),
            "$": command.find(" $")
        }
        
        values = {}
        for key, idx in indices.items():
            if idx != -1:
                start = idx + 3 if key != "X" else idx + 2
                next_key = next((k for k in indices.keys() if indices[k] > idx), None)
                end = indices[next_key] if next_key else len(command)
                values[key] = command[start:end].strip()

        xVal, yVal, zVal, rzVal, ryVal, rxVal, trVal, Speed, ACCspd, DECspd, ACCramp, WC = (
            values[key] for key in ["X", "Y", "Z", "Rz", "Ry", "Rx", "Tr", "S", "Acc", "Dec", "Rm", "$"]
        )

        # Inline getMidPosition logic to get mid position
        curRow = self.tab1.progView.curselection()[0]

        # Inline highlightRow logic for mid position
        last = self.tab1.progView.index('end')
        for r in range(last):
            color = 'dodger blue' if r < curRow else 'black'
            self.tab1.progView.itemconfig(r, {'fg': color})
        self.tab1.progView.itemconfig(curRow, {'fg': 'blue2'})
        self.tab1.progView.selection_clear(0, 'end')
        self.tab1.progView.select_set(curRow)
        self.tab1.progView.see(curRow + 2)

        # Move to next row for mid position
        curRow += 1
        self.tab1.progView.select_set(curRow)
        command = self.tab1.progView.get(curRow).decode()

        # Inline extractPositionValues logic for mid position
        xIndex = command.find(" X ")
        yIndex = command.find(" Y ")
        zIndex = command.find(" Z ")

        Xmid = command[xIndex + 3:yIndex].strip()
        Ymid = command[yIndex + 3:zIndex].strip()
        Zmid = command[zIndex + 3:].strip()

        # Inline getEndPosition logic to get end position
        curRow += 1
        self.tab1.progView.select_set(curRow)
        command = self.tab1.progView.get(curRow).decode()

        # Inline extractPositionValues logic for end position
        xIndex = command.find(" X ")
        yIndex = command.find(" Y ")
        zIndex = command.find(" Z ")

        Xend = command[xIndex + 3:yIndex].strip()
        Yend = command[yIndex + 3:zIndex].strip()
        Zend = command[zIndex + 3:].strip()

        # Inline sendMJCommand logic
        mj_command = f"MJX{Xmid}Y{Ymid}Z{Zmid}Rz{rzVal}Ry{ryVal}Rx{rxVal}Tr{trVal}S{Speed}Ac{ACCspd}Dc{DECspd}Rm{ACCramp}W{WC}\n"
        self.cmdSentEntryField.delete(0, 'end')
        self.cmdSentEntryField.insert(0, mj_command)
        self.ser.write(mj_command.encode())
        self.ser.flushInput()
        time.sleep(0.1)
        self.ser.readline().strip()

        # Inline sendMCCommand logic
        mc_command = f"MC Cx{xVal}Cy{yVal}Cz{zVal}Rz{rzVal}Ry{ryVal}Rx{rxVal}Bx{Xmid}By{Ymid}Bz{Zmid}Px{Xend}Py{Yend}Pz{Zend}Tr{trVal}S{Speed}Ac{ACCspd}Dc{DECspd}Rm{ACCramp}W{WC}\n"
        self.cmdSentEntryField.delete(0, 'end')
        self.cmdSentEntryField.insert(0, mc_command)
        self.ser.write(mc_command.encode())
        self.ser.flushInput()
        time.sleep(0.1)
        self.ser.readline().strip()

    def startSpline(self):
        # Set spline active and update moveInProc status
        self.splineActive = "1"
        if self.moveInProc == 1:
            self.moveInProc = 2

        # Define and send command
        command = "SL\n"
        self.cmdSentEntryField.delete(0, 'end')
        self.cmdSentEntryField.insert(0, command)
        self.ser.write(command.encode())
        self.ser.flushInput()
        
        # Delay and return the response
        time.sleep(0.1)
        return str(self.ser.readline().strip(), 'utf-8')

    def endSpline(self):
        # Set spline inactive and handle queue stop condition
        self.splineActive = "0"
        if self.stopQueue == "1":
            self.stopQueue = "0"
            stop()

        if self.moveInProc == 1:
            self.moveInProc = 2

        # Define and send command
        command = "SS\n"
        self.cmdSentEntryField.delete(0, 'end')
        self.cmdSentEntryField.insert(0, command)
        self.ser.write(command.encode())
        self.ser.flushInput()
        time.sleep(0.1)
        
        # Read and process response
        response = str(self.ser.readline().strip(), 'utf-8')
        
        # Check for error in response and handle accordingly
        if response[:1] == 'E':
            # Handle error by displaying the error message
            print(f"Error: {response}")  # Log the error
            self.errorStatusLabel.configure(text=response, style="Error.TLabel")
        else:
            # Display position data based on response received from the device
            def extractValue(response, label):
                index = response.find(f"{label}")
                if index == -1:
                    raise ValueError(f"Label '{label}' not found in response.")
                start = index + len(label)
                end = response.find(" ", start)
                return response[start:end] if end != -1 else response[start:]

            try:
                # Extract values from response
                xVal = extractValue(response, "X")
                yVal = extractValue(response, "Y")
                zVal = extractValue(response, "Z")
                rzVal = extractValue(response, "Rz")
                ryVal = extractValue(response, "Ry")
                rxVal = extractValue(response, "Rx")

                # Update the UI fields
                self.PositionXField.delete(0, 'end')
                self.PositionXField.insert(0, xVal)
                self.PositionYField.delete(0, 'end')
                self.PositionYField.insert(0, yVal)
                self.PositionZField.delete(0, 'end')
                self.PositionZField.insert(0, zVal)
                self.PositionRzField.delete(0, 'end')
                self.PositionRzField.insert(0, rzVal)
                self.PositionRyField.delete(0, 'end')
                self.PositionRyField.insert(0, ryVal)
                self.PositionRxField.delete(0, 'end')
                self.PositionRxField.insert(0, rxVal)

            except Exception as e:
                # Handle display error
                errorMsg = f"Failed to display position: {str(e)}"
                print(f"Error: {errorMsg}")  # Log the error
                self.errorStatusLabel.configure(text=errorMsg, style="Error.TLabel")

    def cameraOn(self):
        if self.moveInProc == 1:
            self.moveInProc = 2
        self.start_vid()

    def cameraOff(self):
        if self.moveInProc == 1:
            self.moveInProc = 2
        self.stop_vid()

    def visionFind(self, command):
        # Extract necessary values from the command
        templateIndex = command.find("Vis Find - ")
        bgColorIndex = command.find(" - BGcolor ")
        scoreIndex = command.find(" Score ")
        passIndex = command.find(" Pass ")
        failIndex = command.find(" Fail ")

        # Inline extractVisionFindValues logic
        template = command[templateIndex + 11:bgColorIndex]
        checkBG = command[bgColorIndex + 11:scoreIndex]
        
        # Determine background color
        background = "Auto" if checkBG == "(Auto)" else eval(command[bgColorIndex + 11:scoreIndex])
        
        # Extract minimum score
        min_score = float(command[scoreIndex + 7:passIndex]) * 0.01

        # Take a picture and find vision status
        self.take_pic()
        status = self.visFind(template, min_score, background)

        # Handle pass/fail outcomes
        if status == "pass":
            # Inline updateTabSelection logic for pass case
            tabNum = f"Tab Number {command[passIndex + 6:failIndex]}\r\n".encode('utf-8')
            index = self.tab1.progView.get(0, "end").index(tabNum)
            self.tab1.progView.selection_clear(0, 'end')
            self.tab1.progView.select_set(index)

        elif status == "fail":
            # Inline updateTabSelection logic for fail case
            tabNum = f"Tab Number {command[failIndex + 6:]}\r\n".encode('utf-8')
            index = self.tab1.progView.get(0, "end").index(tabNum)
            self.tab1.progView.selection_clear(0, 'end')
            self.tab1.progView.select_set(index)

    # Button jogging #

    def xbox(self):
        def update_status(label_text, label_style="Warn.TLabel"):
            self.almStatusLab.configure(text=label_text, style=label_style)
            self.almStatusLab2.configure(text=label_text, style=label_style)

        def toggle_xbox():
            if self.xboxUse == 0:
                self.xboxUse = 1
                self.mainMode, self.jogMode, self.grip = 1, 1, 0
                update_status('JOGGING JOINTS 1 & 2')
                self.xbcStatusLab.configure(text='Xbox ON')
                self.ChgDis(2)
            else:
                self.xboxUse = 0
                update_status('XBOX CONTROLLER OFF')
                self.xbcStatusLab.configure(text='Xbox OFF')

        def handle_event(event):
            increment = float(self.incrementEntryField.get())

            if event.code == 'ABS_RZ' and event.state >= 100:
                self.ChgDis(0)
            elif event.code == 'ABS_Z' and event.state >= 100:
                self.ChgDis(1)
            elif event.code == 'BTN_TR' and event.state == 1:
                self.ChgSpd(0)
            elif event.code == 'BTN_TL' and event.state == 1:
                self.ChgSpd(1)
            elif event.code == 'BTN_WEST' and event.state == 1:
                self.jogMode = handle_joint_mode(self.mainMode, self.jogMode)
            elif self.mainMode == 1:
                handle_joint_jog(event, self.jogMode, increment)
            elif event.code == 'BTN_SOUTH' and event.state == 1:
                self.jogMode = handle_cartesian_dir_mode(self.mainMode, self.jogMode)
            elif self.mainMode == 2:
                handle_cartesian_dir_jog(event, self.jogMode, increment)
            elif event.code == 'BTN_EAST' and event.state == 1:
                self.jogMode = handle_cartesian_orientation_mode(self.mainMode, self.jogMode)
            elif self.mainMode == 3:
                handle_cartesian_orientation_jog(event, self.jogMode, increment)
            elif event.code == 'BTN_START' and event.state == 1:
                self.mainMode = 4
                update_status('JOGGING TRACK')
            elif self.mainMode == 4:
                handle_track_jog(event, increment)
            elif event.code == 'BTN_NORTH' and event.state == 1:
                self.teachInsertBelSelected()
            elif event.code == 'BTN_SELECT' and event.state == 1:
                handle_gripper(self.grip)
                self.grip = 1 - self.grip

        def handle_joint_mode(main_mode, jog_mode):
            if main_mode != 1:
                update_status('JOGGING JOINTS 1 & 2')
                return 1
            jog_mode = (jog_mode % 3) + 1
            update_status(f'JOGGING JOINTS {2 * jog_mode - 1} & {2 * jog_mode}')
            return jog_mode

        def handle_cartesian_dir_mode(main_mode, jog_mode):
            if main_mode != 2:
                update_status('JOGGING X & Y AXIS')
                return 1
            jog_mode = (jog_mode % 2) + 1
            update_status('JOGGING Z AXIS' if jog_mode == 2 else 'JOGGING X & Y AXIS')
            return jog_mode

        def handle_cartesian_orientation_mode(main_mode, jog_mode):
            if main_mode != 3:
                update_status('JOGGING Rx & Ry AXIS')
                return 1
            jog_mode = (jog_mode % 2) + 1
            update_status('JOGGING Rz AXIS' if jog_mode == 2 else 'JOGGING Rx & Ry AXIS')
            return jog_mode

        def handle_joint_jog(event, jog_mode, increment):
            joint_mapping = {
                (1, 'ABS_HAT0X', 1): self.J1jogNeg,
                (1, 'ABS_HAT0X', -1): self.J1jogPos,
                (1, 'ABS_HAT0Y', -1): self.J2jogNeg,
                (1, 'ABS_HAT0Y', 1): self.J2jogPos,
                (2, 'ABS_HAT0Y', -1): self.J3jogNeg,
                (2, 'ABS_HAT0Y', 1): self.J3jogPos,
                (2, 'ABS_HAT0X', 1): self.J4jogNeg,
                (2, 'ABS_HAT0X', -1): self.J4jogPos,
                (3, 'ABS_HAT0Y', -1): self.J5jogNeg,
                (3, 'ABS_HAT0Y', 1): self.J5jogPos,
                (3, 'ABS_HAT0X', 1): self.J6jogNeg,
                (3, 'ABS_HAT0X', -1): self.J6jogPos
            }
            action = joint_mapping.get((jog_mode, event.code, event.state))
            if action:
                action(increment)

        def handle_cartesian_dir_jog(event, jog_mode, increment):
            cartesian_mapping = {
                (1, 'ABS_HAT0Y', -1): self.XjogNeg,
                (1, 'ABS_HAT0Y', 1): self.XjogPos,
                (1, 'ABS_HAT0X', 1): self.YjogNeg,
                (1, 'ABS_HAT0X', -1): self.YjogPos,
                (2, 'ABS_HAT0Y', 1): self.ZjogNeg,
                (2, 'ABS_HAT0Y', -1): self.ZjogPos
            }
            action = cartesian_mapping.get((jog_mode, event.code, event.state))
            if action:
                action(increment)

        def handle_cartesian_orientation_jog(event, jog_mode, increment):
            orientation_mapping = {
                (1, 'ABS_HAT0X', -1): self.RxjogNeg,
                (1, 'ABS_HAT0X', 1): self.RxjogPos,
                (1, 'ABS_HAT0Y', 1): self.RyjogNeg,
                (1, 'ABS_HAT0Y', -1): self.RyjogPos,
                (2, 'ABS_HAT0X', 1): self.RzjogNeg,
                (2, 'ABS_HAT0X', -1): self.RzjogPos
            }
            action = orientation_mapping.get((jog_mode, event.code, event.state))
            if action:
                action(increment)

        def handle_track_jog(event, increment):
            if event.code == 'ABS_HAT0X' and event.state == 1:
                self.J7jogPos(increment)
            elif event.code == 'ABS_HAT0X' and event.state == -1:
                self.J7jogNeg(increment)

        def handle_gripper(grip_state):
            outputNum = self.DO1offEntryField.get() if grip_state == 0 else self.DO1onEntryField.get()
            command = ("OFX" if grip_state == 0 else "ONX") + outputNum + "\n"
            self.ser2.write(command.encode())
            self.ser2.flushInput()
            time.sleep(0.1)
            self.ser2.read()

        def threadxbox():
            toggle_xbox()
            while self.xboxUse == 1:
                try:
                    events = get_gamepad()
                    for event in events:
                        handle_event(event)
                except:
                    update_status('XBOX CONTROLLER NOT RESPONDING', "Alarm.TLabel")

        threading.Thread(target=threadxbox).start()

    def ChgDis(self, val):
        def increase_speed(cur_speed, increment):
            return min(cur_speed + increment, 100)

        def decrease_speed(cur_speed, decrement):
            return max(cur_speed - decrement, 1)

        cur_spd = int(self.incrementEntryField.get())
        
        if val == 0:  # Increase speed
            cur_spd = increase_speed(cur_spd, 1 if cur_spd < 5 else 5)
        elif val == 1:  # Decrease speed
            cur_spd = decrease_speed(cur_spd, 1 if cur_spd <= 5 else 5)
        elif val == 2:  # Set speed to minimum threshold
            cur_spd = 5

        self.incrementEntryField.delete(0, 'end')
        self.incrementEntryField.insert(0, str(cur_spd))

        time.sleep(0.3)

    def ChgSpd(self, val):
        def increase_speed(cur_speed, increment):
            return min(cur_speed + increment, 100)

        def decrease_speed(cur_speed, decrement):
            return max(cur_speed - decrement, 1)

        cur_spd = int(self.speedEntryField.get())
        
        if val == 0:  # Increase speed
            cur_spd = increase_speed(cur_spd, 1 if cur_spd < 5 else 5)
        elif val == 1:  # Decrease speed
            cur_spd = decrease_speed(cur_spd, 1 if cur_spd <= 5 else 5)
        elif val == 2:  # Set speed to minimum threshold
            cur_spd = 5

        self.speedEntryField.delete(0, 'end')
        self.speedEntryField.insert(0, str(cur_spd))

    def jog_joint(self, joint_index, value, direction):

        # Adjust the appropriate joint angle or position based on the index and direction
        joint_values = [self.J1AngCur, self.J2AngCur, self.J3AngCur, self.J4AngCur, self.J5AngCur, self.J6AngCur, self.J7PosCur, self.J8PosCur, self.J9PosCur]
        if direction == "neg":
            joint_values[joint_index - 1] = str(float(joint_values[joint_index - 1]) - value)
        elif direction == "pos":
            joint_values[joint_index - 1] = str(float(joint_values[joint_index - 1]) + value)
        
        # Update the "SYSTEM READY" status
        self.checkSpeedVals()
        if self.xboxUse != 1:
            self.almStatusLab.configure(text="SYSTEM READY", style="OK.TLabel")
            self.almStatusLab2.configure(text="SYSTEM READY", style="OK.TLabel")

        # Determine speed type and prefix
        speedtype = self.speedOption.get()
        speedPrefix = "Sp" if speedtype in ["mm per Sec", "Percent"] else "Ss"
        if speedtype == "mm per Sec":
            self.speedEntryField.delete(0, 'end')
            self.speedEntryField.insert(0, "50")

        # Get values from entry fields
        Speed = self.speedEntryField.get()
        ACCspd = self.ACCspeedField.get()
        DECspd = self.DECspeedField.get()
        ACCramp = self.ACCrampField.get()
        LoopMode = "".join(str(stat.get()) for stat in [self.J1OpenLoopStat, self.J2OpenLoopStat, self.J3OpenLoopStat, self.J4OpenLoopStat, self.J5OpenLoopStat, self.J6OpenLoopStat])

        # Construct the command string
        command = f"RJ" + "".join(f"{axis}{angle}" for axis, angle in zip("ABCDEF", joint_values[:6])) + \
                f"J7{joint_values[6]}J8{joint_values[7]}J9{joint_values[8]}{speedPrefix}{Speed}Ac{ACCspd}Dc{DECspd}Rm{ACCramp}W{self.WC}Lm{LoopMode}\n"

        # Update the command entry field and send the command
        self.cmdSentEntryField.delete(0, 'end')
        self.cmdSentEntryField.insert(0, command)
        self.ser.write(command.encode())
        self.ser.flushInput()
        time.sleep(.1)
        
        # Read and process response
        response = str(self.ser.readline().strip(), 'utf-8')
        if response.startswith('E'):
            self.ErrorHandler(response)
        else:
            self.displayPosition(response)

    # Define specific jog functions for each joint and direction by calling the helper function
    def J1jogNeg(self, value): self.jog_joint(1, value, "neg")
    def J1jogPos(self, value): self.jog_joint(1, value, "pos")
    def J2jogNeg(self, value): self.jog_joint(2, value, "neg")
    def J2jogPos(self, value): self.jog_joint(2, value, "pos")
    def J3jogNeg(self, value): self.jog_joint(3, value, "neg")
    def J3jogPos(self, value): self.jog_joint(3, value, "pos")
    def J4jogNeg(self, value): self.jog_joint(4, value, "neg")
    def J4jogPos(self, value): self.jog_joint(4, value, "pos")
    def J5jogNeg(self, value): self.jog_joint(5, value, "neg")
    def J5jogPos(self, value): self.jog_joint(5, value, "pos")
    def J6jogNeg(self, value): self.jog_joint(6, value, "neg")
    def J6jogPos(self, value): self.jog_joint(6, value, "pos")
    def J7jogNeg(self, value): self.jog_joint(7, value, "neg")
    def J7jogPos(self, value): self.jog_joint(7, value, "pos")
    def J8jogNeg(self, value): self.jog_joint(8, value, "neg")
    def J8jogPos(self, value): self.jog_joint(8, value, "pos")
    def J9jogNeg(self, value): self.jog_joint(9, value, "neg")
    def J9jogPos(self, value): self.jog_joint(9, value, "pos")

    def LiveJointJog(self, value):
        # Update status labels
        self.almStatusLab.configure(text="SYSTEM READY", text_color="green")
        self.almStatusLab2.configure(text="SYSTEM READY", text_color="green")

        # Check and adjust speed settings
        self.checkSpeedVals()
        speedtype = self.speedOption.get()

        # Ensure speed type is valid and set speed prefix and value
        speedPrefix = "Sp" if speedtype in ["Percent", "mm per Sec"] else "Ss"
        if speedtype == "mm per Sec":
            self.speedOption.set("Percent")
            self.speedEntryField.delete(0, 'end')
            self.speedEntryField.insert(0, "50")

        # Retrieve current values from entry fields
        Speed = self.speedEntryField.get()
        ACCspd = self.ACCspeedField.get()
        DECspd = self.DECspeedField.get()
        ACCramp = self.ACCrampField.get()
        LoopMode = "".join(str(joint.get()) for joint in [
            self.J1OpenLoopStat, self.J2OpenLoopStat, self.J3OpenLoopStat,
            self.J4OpenLoopStat, self.J5OpenLoopStat, self.J6OpenLoopStat
        ])

        # Construct and send command
        command = f"LJV{value}{speedPrefix}{Speed}Ac{ACCspd}Dc{DECspd}Rm{ACCramp}W{self.WC}Lm{LoopMode}\n"
        self.ser.write(command.encode())
        
        # Update command sent field and read response
        self.cmdSentEntryField.delete(0, 'end')
        self.cmdSentEntryField.insert(0, command)
        self.ser.flushInput()
        time.sleep(0.1)
        self.ser.read()

    def LiveCarJog(self, value):

        # Update status labels
        self.almStatusLab.configure(text="SYSTEM READY", text_color="green")
        self.almStatusLab2.configure(text="SYSTEM READY", text_color="green")

        # Check and adjust speed settings
        self.checkSpeedVals()
        speedtype = self.speedOption.get()

        # Ensure speed type is valid and set speed prefix and value
        speedPrefix = "Sp" if speedtype in ["Percent", "mm per Sec"] else "Ss"
        if speedtype == "mm per Sec":
            self.speedOption.set("Percent")
            self.speedEntryField.delete(0, 'end')
            self.speedEntryField.insert(0, "50")

        # Retrieve current values from entry fields
        Speed = self.speedEntryField.get()
        ACCspd = self.ACCspeedField.get()
        DECspd = self.DECspeedField.get()
        ACCramp = self.ACCrampField.get()
        LoopMode = "".join(str(joint.get()) for joint in [
            self.J1OpenLoopStat, self.J2OpenLoopStat, self.J3OpenLoopStat,
            self.J4OpenLoopStat, self.J5OpenLoopStat, self.J6OpenLoopStat
        ])

        # Construct and send command
        command = f"LCV{value}{speedPrefix}{Speed}Ac{ACCspd}Dc{DECspd}Rm{ACCramp}W{self.WC}Lm{LoopMode}\n"
        self.ser.write(command.encode())
        
        # Update command sent field and read response
        self.cmdSentEntryField.delete(0, 'end')
        self.cmdSentEntryField.insert(0, command)
        self.ser.flushInput()
        time.sleep(0.1)
        self.ser.read()

    def LiveToolJog(self, value):

        # Update status labels
        self.almStatusLab.configure(text="SYSTEM READY", text_color="green")
        self.almStatusLab2.configure(text="SYSTEM READY", text_color="green")

        # Check and adjust speed settings
        self.checkSpeedVals()
        speedtype = self.speedOption.get()

        # Ensure speed type is valid and set speed prefix and value
        speedPrefix = "Sp" if speedtype in ["Percent", "mm per Sec"] else "Ss"
        if speedtype == "mm per Sec":
            self.speedOption.set("Percent")
            self.speedEntryField.delete(0, 'end')
            self.speedEntryField.insert(0, "50")

        # Retrieve current values from entry fields
        Speed = self.speedEntryField.get()
        ACCspd = self.ACCspeedField.get()
        DECspd = self.DECspeedField.get()
        ACCramp = self.ACCrampField.get()
        LoopMode = "".join(str(joint.get()) for joint in [
            self.J1OpenLoopStat, self.J2OpenLoopStat, self.J3OpenLoopStat,
            self.J4OpenLoopStat, self.J5OpenLoopStat, self.J6OpenLoopStat
        ])

        # Construct and send command
        command = f"LTV{value}{speedPrefix}{Speed}Ac{ACCspd}Dc{DECspd}Rm{ACCramp}W{self.WC}Lm{LoopMode}\n"
        self.ser.write(command.encode())
        
        # Update command sent field and read response
        self.cmdSentEntryField.delete(0, 'end')
        self.cmdSentEntryField.insert(0, command)
        self.ser.flushInput()
        time.sleep(0.1)
        self.ser.read()

    def StopJog(self):
        command = "S\n"
        if int(self.IncJogStat.get()) == 0:
            self.ser.write(command.encode())
            self.ser.flushInput()
            time.sleep(0.1)
            
            # Read and handle response
            response = self.ser.readline().decode('utf-8').strip()
            if response.startswith('E'):
                self.ErrorHandler(response)
            else:
                self.displayPosition(response)

    def jog_joint_command(self, joint_index, value, direction):

        self.checkSpeedVals()
        if self.xboxUse != 1:
            self.almStatusLab.configure(text="SYSTEM READY", style="OK.TLabel")
            self.almStatusLab2.configure(text="SYSTEM READY", style="OK.TLabel")

        # Determine speed prefix and set default speed if needed
        speedtype = self.speedOption.get()
        if speedtype == "mm per Sec":
            speedPrefix = "Sp"
            self.speedEntryField.delete(0, 'end')
            self.speedEntryField.insert(0, "50")
        elif speedtype == "Seconds":
            speedPrefix = "Ss"
        else:
            speedPrefix = "Sp"

        Speed = self.speedEntryField.get()
        ACCspd = self.ACCspeedField.get()
        DECspd = self.DECspeedField.get()
        ACCramp = self.ACCrampField.get()
        LoopMode = "".join(str(stat.get()) for stat in [self.J1OpenLoopStat, self.J2OpenLoopStat, self.J3OpenLoopStat, self.J4OpenLoopStat, self.J5OpenLoopStat, self.J6OpenLoopStat])

        # Adjust joint position based on direction
        joint_positions = [self.J1AngCur, self.J2AngCur, self.J3AngCur, self.J4AngCur, self.J5AngCur, self.J6AngCur, self.J7PosCur, self.J8PosCur, self.J9PosCur]
        if direction == "neg":
            joint_positions[joint_index - 1] = str(float(joint_positions[joint_index - 1]) - value)
        else:
            joint_positions[joint_index - 1] = str(float(joint_positions[joint_index - 1]) + value)

        # Build command string
        command = f"RJ" + "".join(f"{axis}{pos}" for axis, pos in zip("ABCDEF", joint_positions[:6])) + \
                f"J7{joint_positions[6]}J8{joint_positions[7]}J9{joint_positions[8]}{speedPrefix}{Speed}" + \
                f"Ac{ACCspd}Dc{DECspd}Rm{ACCramp}W{self.WC}Lm{LoopMode}\n"

        # Send command to serial
        self.ser.write(command.encode())
        self.cmdSentEntryField.delete(0, 'end')
        self.cmdSentEntryField.insert(0, command)
        self.ser.flushInput()
        time.sleep(.1)

        # Process response
        response = str(self.ser.readline().strip(), 'utf-8')
        if response.startswith('E'):
            self.ErrorHandler(response)
        else:
            self.displayPosition(response)

    # Define jog functions for J7, J8, and J9 using the helper function
    def J7jogNeg(self, value): self.jog_joint_command(7, value, "neg")
    def J7jogPos(self, value): self.jog_joint_command(7, value, "pos")
    def J8jogNeg(self, value): self.jog_joint_command(8, value, "neg")
    def J8jogPos(self, value): self.jog_joint_command(8, value, "pos")
    def J9jogNeg(self, value): self.jog_joint_command(9, value, "neg")
    def J9jogPos(self, value): self.jog_joint_command(9, value, "pos")

    def jog_neg_with_command(self, axis, value):
        # Get speed prefix based on speed option
        speedtype = self.speedOption.get()
        speedPrefix = ""
        if speedtype == "mm per Sec":
            speedPrefix = "Sm"
        elif speedtype == "Seconds":
            speedPrefix = "Ss"
        elif speedtype == "Percent":
            speedPrefix = "Sp"

        # Retrieve necessary speed values
        Speed = self.speedEntryField.get()
        ACCspd = self.ACCspeedField.get()
        DECspd = self.DECspeedField.get()
        ACCramp = self.ACCrampField.get()
        j7Val = str(self.J7PosCur)
        j8Val = str(self.J8PosCur)
        j9Val = str(self.J9PosCur)
        
        # Compile loop mode status for each joint
        LoopMode = ''.join([
            str(self.J1OpenLoopStat.get()), str(self.J2OpenLoopStat.get()), str(self.J3OpenLoopStat.get()),
            str(self.J4OpenLoopStat.get()), str(self.J5OpenLoopStat.get()), str(self.J6OpenLoopStat.get())
        ])
        
        # Check if Xbox controller is in use
        self.checkSpeedVals()
        if self.xboxUse != 1:
            self.almStatusLab.configure(text="SYSTEM READY", style="OK.TLabel")
            self.almStatusLab2.configure(text="SYSTEM READY", style="OK.TLabel")
        
        # Calculate new positions based on axis and value
        xVal = self.XcurPos if axis != 'X' else str(float(self.XcurPos) - value)
        yVal = self.YcurPos if axis != 'Y' else str(float(self.YcurPos) - value)
        zVal = self.ZcurPos if axis != 'Z' else str(float(self.ZcurPos) - value)
        rzVal = self.RzcurPos if axis != 'Rz' else str(float(self.RzcurPos) - value)
        ryVal = self.RycurPos if axis != 'Ry' else str(float(self.RycurPos) - value)
        rxVal = self.RxcurPos if axis != 'Rx' else str(float(self.RxcurPos) - value)

        # Generate command string
        command = (
            f"MJX{xVal}Y{yVal}Z{zVal}Rz{rzVal}Ry{ryVal}Rx{rxVal}J7{j7Val}J8{j8Val}"
            f"J9{j9Val}{speedPrefix}{Speed}Ac{ACCspd}Dc{DECspd}Rm{ACCramp}W{self.WC}Lm{LoopMode}\n"
        )

        # Send command and handle response
        self.ser.write(command.encode())
        self.cmdSentEntryField.delete(0, 'end')
        self.cmdSentEntryField.insert(0, command)
        self.ser.flushInput()
        time.sleep(0.1)
        response = str(self.ser.readline().strip(), 'utf-8')
        if response.startswith('E'):
            self.ErrorHandler(response)
        else:
            self.displayPosition(response)

    # Updated jog functions to use the consolidated function
    def XjogNeg(self, value): self.jog_neg_with_command('X', value)
    def YjogNeg(self, value): self.jog_neg_with_command('Y', value)
    def ZjogNeg(self, value): self.jog_neg_with_command('Z', value)
    def RxjogNeg(self, value): self.jog_neg_with_command('Rx', value)
    def RyjogNeg(self, value): self.jog_neg_with_command('Ry', value)
    def RzjogNeg(self, value): self.jog_neg_with_command('Rz', value)

    def jog_pos_with_command(self, axis, value):
        # Get speed prefix based on speed option
        speedtype = self.speedOption.get()
        speedPrefix = ""
        if speedtype == "mm per Sec":
            speedPrefix = "Sm"
        elif speedtype == "Seconds":
            speedPrefix = "Ss"
        elif speedtype == "Percent":
            speedPrefix = "Sp"

        # Retrieve necessary speed values
        Speed = self.speedEntryField.get()
        ACCspd = self.ACCspeedField.get()
        DECspd = self.DECspeedField.get()
        ACCramp = self.ACCrampField.get()
        j7Val = str(self.J7PosCur)
        j8Val = str(self.J8PosCur)
        j9Val = str(self.J9PosCur)
        
        # Compile loop mode status for each joint
        LoopMode = ''.join([
            str(self.J1OpenLoopStat.get()), str(self.J2OpenLoopStat.get()), str(self.J3OpenLoopStat.get()),
            str(self.J4OpenLoopStat.get()), str(self.J5OpenLoopStat.get()), str(self.J6OpenLoopStat.get())
        ])
        
        # Check if Xbox controller is in use
        self.checkSpeedVals()
        if self.xboxUse != 1:
            self.almStatusLab.configure(text="SYSTEM READY", style="OK.TLabel")
            self.almStatusLab2.configure(text="SYSTEM READY", style="OK.TLabel")
        
        # Calculate new positions based on axis and value for positive jog
        xVal = self.XcurPos if axis != 'X' else str(float(self.XcurPos) + value)
        yVal = self.YcurPos if axis != 'Y' else str(float(self.YcurPos) + value)
        zVal = self.ZcurPos if axis != 'Z' else str(float(self.ZcurPos) + value)
        rzVal = self.RzcurPos if axis != 'Rz' else str(float(self.RzcurPos) + value)
        ryVal = self.RycurPos if axis != 'Ry' else str(float(self.RycurPos) + value)
        rxVal = self.RxcurPos if axis != 'Rx' else str(float(self.RxcurPos) + value)

        # Generate command string
        command = (
            f"MJX{xVal}Y{yVal}Z{zVal}Rz{rzVal}Ry{ryVal}Rx{rxVal}J7{j7Val}J8{j8Val}"
            f"J9{j9Val}{speedPrefix}{Speed}Ac{ACCspd}Dc{DECspd}Rm{ACCramp}W{self.WC}Lm{LoopMode}\n"
        )

        # Send command and handle response
        self.ser.write(command.encode())
        self.cmdSentEntryField.delete(0, 'end')
        self.cmdSentEntryField.insert(0, command)
        self.ser.flushInput()
        time.sleep(0.1)
        response = str(self.ser.readline().strip(), 'utf-8')
        if response.startswith('E'):
            self.ErrorHandler(response)
        else:
            self.displayPosition(response)

    # Updated jog functions to use the consolidated function for positive jogs
    def XjogPos(self, value): self.jog_pos_with_command('X', value)
    def YjogPos(self, value): self.jog_pos_with_command('Y', value)
    def ZjogPos(self, value): self.jog_pos_with_command('Z', value)
    def RxjogPos(self, value): self.jog_pos_with_command('Rx', value)
    def RyjogPos(self, value): self.jog_pos_with_command('Ry', value)
    def RzjogPos(self, value): self.jog_pos_with_command('Rz', value)

    def execute_t_jog_neg(self, axis_prefix, value):
        # Check and set system readiness
        self.checkSpeedVals()
        if self.xboxUse != 1:
            self.almStatusLab.configure(text="SYSTEM READY", fg_color="green")
            self.almStatusLab2.configure(text="SYSTEM READY", fg_color="green")

        # Handle speed settings, restricting "mm per Sec" if needed
        speedtype = self.speedOption.get()
        if speedtype == "mm per Sec":
            self.speedOption.configure(values=["Percent", "Seconds", "mm per Sec"])
            self.speedOption.set("Percent")
            speedPrefix = "Ss"
            self.speedEntryField.delete(0, 'end')
            self.speedEntryField.insert(0, "50")
        elif speedtype == "Seconds":
            speedPrefix = "Ss"
        elif speedtype == "Percent":
            speedPrefix = "Sp"
        else:
            speedPrefix = ""

        # Collect other parameters
        Speed = self.speedEntryField.get()
        ACCspd = self.ACCspeedField.get()
        DECspd = self.DECspeedField.get()
        ACCramp = self.ACCrampField.get()

        # Compile loop mode
        LoopMode = ''.join([
            str(self.J1OpenLoopStat.get()), str(self.J2OpenLoopStat.get()), str(self.J3OpenLoopStat.get()),
            str(self.J4OpenLoopStat.get()), str(self.J5OpenLoopStat.get()), str(self.J6OpenLoopStat.get())
        ])

        # Construct the command for the specific T-axis jog
        command = (
            f"JT{axis_prefix}1{value}{speedPrefix}{Speed}G{ACCspd}H{DECspd}"
            f"I{ACCramp}Lm{LoopMode}\n"
        )

        # Send the command and handle response
        self.ser.write(command.encode())
        self.cmdSentEntryField.delete(0, 'end')
        self.cmdSentEntryField.insert(0, command)
        self.ser.flushInput()
        time.sleep(0.1)
        response = str(self.ser.readline().strip(), 'utf-8')
        if response.startswith('E'):
            self.ErrorHandler(response)
        else:
            self.displayPosition(response)

    # Individual jog functions now call execute_t_jog_neg with their specific axis_prefix
    def TXjogNeg(self, value): self.execute_t_jog_neg('X', value)
    def TYjogNeg(self, value): self.execute_t_jog_neg('Y', value)
    def TZjogNeg(self, value): self.execute_t_jog_neg('Z', value)
    def TRxjogNeg(self, value): self.execute_t_jog_neg('W', value)
    def TRyjogNeg(self, value): self.execute_t_jog_neg('P', value)
    def TRzjogNeg(self, value): self.execute_t_jog_neg('R', value)

    def execute_t_jog_pos(self, axis_prefix, value):
        # Check and set system readiness
        self.checkSpeedVals()
        if self.xboxUse != 1:
            self.almStatusLab.configure(text="SYSTEM READY", fg_color="green")
            self.almStatusLab2.configure(text="SYSTEM READY", fg_color="green")

        # Handle speed settings, restricting "mm per Sec" if needed
        speedtype = self.speedOption.get()
        if speedtype == "mm per Sec":
            self.speedOption.configure(values=["Percent", "Seconds", "mm per Sec"])
            self.speedOption.set("Percent")
            speedPrefix = "Ss"
            self.speedEntryField.delete(0, 'end')
            self.speedEntryField.insert(0, "50")
        elif speedtype == "Seconds":
            speedPrefix = "Ss"
        elif speedtype == "Percent":
            speedPrefix = "Sp"
        else:
            speedPrefix = ""

        # Collect other parameters
        Speed = self.speedEntryField.get()
        ACCspd = self.ACCspeedField.get()
        DECspd = self.DECspeedField.get()
        ACCramp = self.ACCrampField.get()

        # Compile loop mode (fixed typo in J4 access)
        LoopMode = ''.join([
            str(self.J1OpenLoopStat.get()), str(self.J2OpenLoopStat.get()), str(self.J3OpenLoopStat.get()),
            str(self.J4OpenLoopStat.get()), str(self.J5OpenLoopStat.get()), str(self.J6OpenLoopStat.get())
        ])

        # Construct the command for the specific T-axis positive jog
        command = (
            f"JT{axis_prefix}1+{value}{speedPrefix}{Speed}G{ACCspd}H{DECspd}"
            f"I{ACCramp}Lm{LoopMode}\n"
        )

        # Send the command and handle response
        self.ser.write(command.encode())
        self.cmdSentEntryField.delete(0, 'end')
        self.cmdSentEntryField.insert(0, command)
        self.ser.flushInput()
        time.sleep(0.1)
        response = str(self.ser.readline().strip(), 'utf-8')
        if response.startswith('E'):
            self.ErrorHandler(response)
        else:
            self.displayPosition(response)

    # Individual jog functions now call execute_t_jog_pos with their specific axis_prefix
    def TXjogPos(self, value): self.execute_t_jog_pos('X', value)
    def TYjogPos(self, value): self.execute_t_jog_pos('Y', value)
    def TZjogPos(self, value): self.execute_t_jog_pos('Z', value)
    def TRxjogPos(self, value): self.execute_t_jog_pos('W', value)
    def TRyjogPos(self, value): self.execute_t_jog_pos('P', value)
    def TRzjogPos(self, value): self.execute_t_jog_pos('R', value)

    # Teach defs #

    def teachInsertBelSelected(self):
        def get_selected_row(self):
            try:
                sel_row = self.tab1.progView.curselection()[0] + 1
            except:
                last = self.tab1.progView.index('end')
                sel_row = last
                self.tab1.progView.select_set(sel_row)
            return sel_row

        def determine_speed_prefix(speed_type):
            if speed_type == "Seconds":
                return "Ss"
            elif speed_type == "mm per Sec":
                return "Sm"
            elif speed_type == "Percent":
                return "Sp"
            return ""

        def insert_to_view_and_save(self, new_position, sel_row):
            self.tab1.progView.insert(sel_row, bytes(new_position + '\n', 'utf-8'))
            self.tab1.progView.selection_clear(0, 'end')
            self.tab1.progView.select_set(sel_row)
            items = self.tab1.progView.get(0, 'end')
            file_path = path.relpath(self.ProgEntryField.get())
            with open(file_path, 'w', encoding='utf-8') as f:
                for item in items:
                    f.write(str(item.strip(), encoding='utf-8'))
                    f.write('\n')

        # Main function code starts here

        self.check_speed_vals()
        sel_row = get_selected_row()

        Speed = self.speedEntryField.get()
        speed_type = self.speedOption.get()
        speed_prefix = determine_speed_prefix(speed_type)

        ACCspd = self.ACCspeedField.get()
        DECspd = self.DECspeedField.get()
        ACCramp = self.ACCrampField.get()
        Rounding = self.roundEntryField.get()
        movetype = self.options.get()

        # Handle each movetype case and call insert_to_view_and_save accordingly
        if movetype in ["OFF J", "Move Vis", "Move J", "Move L", "Move R", "Move A Mid", "Move A End", "Move C Center"]:
            new_position = (
                f"{movetype} [*] X {self.XcurPos} Y {self.YcurPos} Z {self.ZcurPos} Rz {self.RzcurPos} "
                f"Ry {self.RycurPos} Rx {self.RxcurPos} J7 {self.J7PosCur} J8 {self.J8PosCur} J9 {self.J9PosCur} "
                f"{speed_prefix} {Speed} Ac {ACCspd} Dc {DECspd} Rm {ACCramp} $ {self.WC}"
            )
            insert_to_view_and_save(new_position, sel_row)

        elif movetype == "Move PR":
            new_position = (
                f"{movetype} [ PR: {self.SavePosEntryField.get()} ] [*] J7 {self.J7PosCur} J8 {self.J8PosCur} "
                f"J9 {self.J9PosCur} {speed_prefix} {Speed} Ac {ACCspd} Dc {DECspd} Rm {ACCramp} $ {self.WC}"
            )
            insert_to_view_and_save(new_position, sel_row)

        elif movetype == "OFF PR ":
            new_position = (
                f"{movetype} [ PR: {self.SavePosEntryField.get()} ] offs [ *PR: {int(self.SavePosEntryField.get()) + 1} ] "
                f"[*] J7 {self.J7PosCur} J8 {self.J8PosCur} J9 {self.J9PosCur} {speed_prefix} {Speed} Ac {ACCspd} Dc {DECspd} Rm {ACCramp} $ {self.WC}"
            )
            insert_to_view_and_save(new_position, sel_row)

        elif movetype == "Move C Start":
            new_position = f"{movetype} [*] X {self.XcurPos} Y {self.YcurPos} Z {self.ZcurPos}"
            insert_to_view_and_save(new_position, sel_row)

        elif movetype == "Teach PR":
            PR = str(self.SavePosEntryField.get())
            elements = [
                f"Position Register {PR} Element 6 = {self.RxcurPos}",
                f"Position Register {PR} Element 5 = {self.RycurPos}",
                f"Position Register {PR} Element 4 = {self.RzcurPos}",
                f"Position Register {PR} Element 3 = {self.ZcurPos}",
                f"Position Register {PR} Element 2 = {self.YcurPos}",
                f"Position Register {PR} Element 1 = {self.XcurPos}"
            ]
            for element in elements:
                self.tab1.progView.insert(sel_row, bytes(element + '\n', 'utf-8'))
                sel_row += 1
            insert_to_view_and_save("", sel_row)
    
    def teachReplaceSelected(self):
        try:
            self.deleteitem()
            selRow = self.tab1.progView.curselection()[0]
            self.tab1.progView.select_set(selRow - 1)
        except IndexError:
            selRow = self.tab1.progView.index('end')
            self.tab1.progView.select_set(selRow)

        self.teachInsertBelSelected()

    # Program function defs #

    def deleteitem(self):
        try:
            # Get the currently selected row index
            selRow = self.tab1.progView.curselection()[0]
            
            # Delete the selected item and clear any selection
            self.tab1.progView.delete(selRow)
            self.tab1.progView.selection_clear(0, END)
            
            # Re-select the current row if possible
            self.tab1.progView.select_set(min(selRow, self.tab1.progView.index('end') - 1))

            # Save the updated list of items back to the file
            items = self.tab1.progView.get(0, END)
            file_path = path.relpath(self.ProgEntryField.get())
            with open(file_path, 'w', encoding='utf-8') as f:
                for item in items:
                    f.write(str(item.strip(), encoding='utf-8') + '\n')
        except IndexError:
            # Handle the case when no selection is available
            pass

    def manInsItem(self):
        try:
            sel_row = self.tab1.prog_view.curselection()[0]
            sel_row += 1
        except IndexError:  # handle specific exception for no selection
            last = self.tab1.prog_view.size() - 1
            sel_row = last
            self.tab1.prog_view.select_set(sel_row)
        
        # Insert the item and clear previous selections
        self.tab1.prog_view.insert(sel_row, bytes(self.man_entry_field.get() + '\n', 'utf-8'))
        self.tab1.prog_view.selection_clear(0, ctk.END)
        self.tab1.prog_view.select_set(sel_row)
        
        # Update current row entry
        self.cur_row_entry_field.delete(0, 'end')
        self.cur_row_entry_field.insert(0, sel_row)
        
        # Set item color (in CTk, you may need to manage text color in other ways)
        self.tab1.prog_view.itemconfig(sel_row, {'foreground': 'darkgreen'})
        
        # Write updated list to file
        items = self.tab1.prog_view.get(0, ctk.END)
        file_path = os.path.relpath(self.prog_entry_field.get())
        with open(file_path, 'w', encoding='utf-8') as file:
            for item in items:
                file.write(item.decode('utf-8').strip() + '\n')

    def manReplItem(self):
        # Get the selected row
        try:
            sel_row = self.tab1.prog_view.curselection()[0]
        except IndexError:  # Handle case where no row is selected
            return
        
        # Delete and replace the item at selected row
        self.tab1.prog_view.delete(sel_row)
        self.tab1.prog_view.insert(sel_row, bytes(self.man_entry_field.get() + '\n', 'utf-8'))
        
        # Update selection and clear previous selections
        self.tab1.prog_view.selection_clear(0, ctk.END)
        self.tab1.prog_view.select_set(sel_row)
        
        # Update item color (CTk might need alternative styling if not directly supported)
        self.tab1.prog_view.itemconfig(sel_row, {'foreground': 'darkgreen'})
        
        # Write updated list to file
        items = self.tab1.prog_view.get(0, ctk.END)
        file_path = os.path.relpath(self.prog_entry_field.get())
        with open(file_path, 'w', encoding='utf-8') as file:
            for item in items:
                file.write(item.decode('utf-8').strip() + '\n')

    def waitTime(self):
        try:
            # Get the selected row and increment by 1 to insert below the current selection
            sel_row = self.tab1.prog_view.curselection()[0] + 1
        except IndexError:
            # If no selection, set sel_row to the last position
            sel_row = self.tab1.prog_view.size()
        
        # Prepare the new "Wait Time" text
        seconds = self.wait_time_entry_field.get()
        new_time = f"Wait Time = {seconds}"
        
        # Insert new item in the list
        self.tab1.prog_view.insert(sel_row, bytes(new_time + '\n', 'utf-8'))
        self.tab1.prog_view.selection_clear(0, ctk.END)
        self.tab1.prog_view.select_set(sel_row)
        
        # Write the updated list to the file
        items = self.tab1.prog_view.get(0, ctk.END)
        file_path = os.path.relpath(self.prog_entry_field.get())
        with open(file_path, 'w', encoding='utf-8') as file:
            for item in items:
                file.write(item.decode('utf-8').strip() + '\n')

    def waitInputOn(self):
        try:
            # Get the selected row and increment by 1 to insert below the current selection
            sel_row = self.tab1.prog_view.curselection()[0] + 1
        except IndexError:
            # If no selection, set sel_row to the last position
            sel_row = self.tab1.prog_view.size()
        
        # Prepare the "Wait Input On" text
        input_value = self.wait_input_entry_field.get()
        new_input = f"Wait Input On = {input_value}"
        
        # Insert new item in the list
        self.tab1.prog_view.insert(sel_row, bytes(new_input + '\n', 'utf-8'))
        self.tab1.prog_view.selection_clear(0, ctk.END)
        self.tab1.prog_view.select_set(sel_row)
        
        # Write the updated list to the file
        items = self.tab1.prog_view.get(0, ctk.END)
        file_path = os.path.relpath(self.prog_entry_field.get())
        with open(file_path, 'w', encoding='utf-8') as file:
            for item in items:
                file.write(item.decode('utf-8').strip() + '\n')

    def waitInputOff(self):
        try:
            # Get the selected row and increment by 1 to insert below the current selection
            sel_row = self.tab1.prog_view.curselection()[0] + 1
        except IndexError:
            # If no selection, set sel_row to the last position
            sel_row = self.tab1.prog_view.size()
        
        # Prepare the "Wait Off Input" text
        input_value = self.wait_input_off_entry_field.get()
        new_input = f"Wait Off Input = {input_value}"
        
        # Insert new item in the list
        self.tab1.prog_view.insert(sel_row, bytes(new_input + '\n', 'utf-8'))
        self.tab1.prog_view.selection_clear(0, ctk.END)
        self.tab1.prog_view.select_set(sel_row)
        
        # Write the updated list to the file
        items = self.tab1.prog_view.get(0, ctk.END)
        file_path = os.path.relpath(self.prog_entry_field.get())
        with open(file_path, 'w', encoding='utf-8') as file:
            for item in items:
                file.write(item.decode('utf-8').strip() + '\n')

    def setOutputOn(self):
        try:
            # Get the selected row and increment by 1 to insert below the current selection
            sel_row = self.tab1.prog_view.curselection()[0] + 1
        except IndexError:
            # If no selection, set sel_row to the last position
            sel_row = self.tab1.prog_view.size()
        
        # Prepare the "Out On" text
        output_value = self.output_on_entry_field.get()
        new_output = f"Out On = {output_value}"
        
        # Insert new item in the list
        self.tab1.prog_view.insert(sel_row, bytes(new_output + '\n', 'utf-8'))
        self.tab1.prog_view.selection_clear(0, ctk.END)
        self.tab1.prog_view.select_set(sel_row)
        
        # Write the updated list to the file
        items = self.tab1.prog_view.get(0, ctk.END)
        file_path = os.path.relpath(self.prog_entry_field.get())
        with open(file_path, 'w', encoding='utf-8') as file:
            for item in items:
                file.write(item.decode('utf-8').strip() + '\n')

    def setOutputOff(self):
        try:
            # Get the selected row and increment by 1 to insert below the current selection
            sel_row = self.tab1.prog_view.curselection()[0] + 1
        except IndexError:
            # If no selection, set sel_row to the last position
            sel_row = self.tab1.prog_view.size()
        
        # Prepare the "Out Off" text
        output_value = self.output_off_entry_field.get()
        new_output = f"Out Off = {output_value}"
        
        # Insert new item in the list
        self.tab1.prog_view.insert(sel_row, bytes(new_output + '\n', 'utf-8'))
        self.tab1.prog_view.selection_clear(0, ctk.END)
        self.tab1.prog_view.select_set(sel_row)
        
        # Write the updated list to the file
        items = self.tab1.prog_view.get(0, ctk.END)
        file_path = os.path.relpath(self.prog_entry_field.get())
        with open(file_path, 'w', encoding='utf-8') as file:
            for item in items:
                file.write(item.decode('utf-8').strip() + '\n')

    def tabNumber(self):
        try:
            # Get the selected row and increment by 1 to insert below the current selection
            sel_row = self.tab1.prog_view.curselection()[0] + 1
        except IndexError:
            # If no selection, set sel_row to the last position
            sel_row = self.tab1.prog_view.size()
        
        # Prepare the "Tab Number" text
        tab_num = self.tab_num_entry_field.get()
        tab_insert = f"Tab Number {tab_num}"
        
        # Insert new item in the list
        self.tab1.prog_view.insert(sel_row, bytes(tab_insert + '\n', 'utf-8'))
        self.tab1.prog_view.selection_clear(0, ctk.END)
        self.tab1.prog_view.select_set(sel_row)
        
        # Write the updated list to the file
        items = self.tab1.prog_view.get(0, ctk.END)
        file_path = os.path.relpath(self.prog_entry_field.get())
        with open(file_path, 'w', encoding='utf-8') as file:
            for item in items:
                file.write(item.decode('utf-8').strip() + '\n')

    def jumpTab(self):
        try:
            # Get the selected row and increment by 1 to insert below the current selection
            sel_row = self.tab1.prog_view.curselection()[0] + 1
        except IndexError:
            # If no selection, set sel_row to the last position
            sel_row = self.tab1.prog_view.size()
        
        # Prepare the "Jump Tab" text
        tab_num = self.jump_tab_entry_field.get()
        tab_jump_text = f"Jump Tab-{tab_num}"
        
        # Insert new item in the list
        self.tab1.prog_view.insert(sel_row, bytes(tab_jump_text + '\n', 'utf-8'))
        self.tab1.prog_view.selection_clear(0, ctk.END)
        self.tab1.prog_view.select_set(sel_row)
        
        # Write the updated list to the file
        items = self.tab1.prog_view.get(0, ctk.END)
        file_path = os.path.relpath(self.prog_entry_field.get())
        with open(file_path, 'w', encoding='utf-8') as file:
            for item in items:
                file.write(item.decode('utf-8').strip() + '\n')

    def cameraOn(self):
        try:
            # Get the selected row and increment by 1 to insert below the current selection
            selRow = self.tab1.progView.curselection()[0] + 1
        except IndexError:
            # If no selection, set selRow to the last position
            selRow = self.tab1.progView.size()
        
        # Insert "Cam On" text into the list
        value = "Cam On"
        self.tab1.progView.insert(selRow, bytes(value + '\n', 'utf-8'))
        self.tab1.progView.selection_clear(0, ctk.END)
        self.tab1.progView.select_set(selRow)
        
        # Write the updated list to the file
        items = self.tab1.progView.get(0, ctk.END)
        file_path = os.path.relpath(self.ProgEntryField.get())
        with open(file_path, 'w', encoding='utf-8') as f:
            for item in items:
                f.write(item.decode('utf-8').strip() + '\n')

    def cameraOff(self):
        try:
            selRow = self.tab1.progView.curselection()[0]
            selRow += 1
        except IndexError:
            last = self.tab1.progView.size() - 1
            selRow = last
            self.tab1.progView.select_set(selRow)
        
        value = "Cam Off"
        self.tab1.progView.insert(selRow, bytes(value + '\n', 'utf-8'))
        self.tab1.progView.selection_clear(0, ctk.END)
        self.tab1.progView.select_set(selRow)
        
        items = self.tab1.progView.get(0, ctk.END)
        file_path = os.path.relpath(self.ProgEntryField.get())
        
        with open(file_path, 'w', encoding='utf-8') as f:
            for item in items:
                # Strip and decode each item from bytes back to a string for consistent UTF-8 encoding in the file
                f.write(item.strip().decode('utf-8'))
                f.write('\n')

    def IfCMDInsert(self):
        localErrorFlag = False
        try:
            selRow = self.tab1.progView.curselection()[0]
            selRow += 1
        except IndexError:
            last = self.tab1.progView.size() - 1
            selRow = last
            self.tab1.progView.select_set(selRow)

        option = self.iFoption.get()
        selection = self.iFselection.get()
        variable = self.IfVarEntryField.get()
        
        if not variable:
            localErrorFlag = True
            self.almStatusLab.configure(text="Please enter an input, register number or COM Port", style="Alarm.TLabel")
            
        inputVal = self.IfInputEntryField.get()
        destVal = self.IfDestEntryField.get()
        prefix = ""

        if option == "Input":
            if inputVal in ["0", "1"]:
                prefix = f"If Input # {variable} = {inputVal} :"
            else:
                localErrorFlag = True
                self.almStatusLab.configure(text="Please enter a 1 or 0 for the = value", style="Alarm.TLabel")
        
        elif option == "Register":
            if not inputVal:
                localErrorFlag = True
                self.almStatusLab.configure(text="Please enter a register number", style="Alarm.TLabel")
            prefix = f"If Register # {variable} = {inputVal} :"

        elif option == "COM Device":
            if not inputVal:
                localErrorFlag = True
                self.almStatusLab.configure(text="Please enter expected COM device input", style="Alarm.TLabel")
            prefix = f"If COM Device # {variable} = {inputVal} :"
        
        if selection == "Call Prog":
            if not destVal:
                localErrorFlag = True
                self.almStatusLab.configure(text="Please enter a program name", style="Alarm.TLabel")
            value = f"{prefix} Call Prog {destVal}"
        
        elif selection == "Jump Tab":
            if not destVal:
                localErrorFlag = True
                self.almStatusLab.configure(text="Please enter a destination tab", style="Alarm.TLabel")
            value = f"{prefix} Jump to Tab {destVal}"
        
        if not localErrorFlag:
            self.tab1.progView.insert(selRow, bytes(value + '\n', 'utf-8'))
            self.tab1.progView.selection_clear(0, ctk.END)
            self.tab1.progView.select_set(selRow)
            
            items = self.tab1.progView.get(0, ctk.END)
            file_path = os.path.relpath(self.ProgEntryField.get())
            
            with open(file_path, 'w', encoding='utf-8') as f:
                for item in items:
                    f.write(item.strip().decode('utf-8'))
                    f.write('\n')

    def ReadAuxCom(self):
        try:
            selRow = self.tab1.progView.curselection()[0]
            selRow += 1
        except IndexError:
            last = self.tab1.progView.size() - 1
            selRow = last
            self.tab1.progView.select_set(selRow)

        comNum = self.auxPortEntryField.get()
        comChar = self.auxCharEntryField.get()
        servoins = f"Read COM # {comNum} Char: {comChar}"
        
        self.tab1.progView.insert(selRow, bytes(servoins + '\n', 'utf-8'))
        self.tab1.progView.selection_clear(0, ctk.END)
        self.tab1.progView.select_set(selRow)
        
        items = self.tab1.progView.get(0, ctk.END)
        file_path = os.path.relpath(self.ProgEntryField.get())
        
        with open(file_path, 'w', encoding='utf-8') as f:
            for item in items:
                f.write(item.strip().decode('utf-8'))
                f.write('\n')

    def TestAuxCom(self):
        try:
            port = f"COM{self.com3PortEntryField.get()}"
            baud = 115200
            self.ser3 = serial.Serial(port, baud, timeout=5)
            self.ser3.flushInput()
        except serial.SerialException:
            Curtime = datetime.datetime.now().strftime("%B %d %Y - %I:%M%p")
            error_message = f"{Curtime} - UNABLE TO ESTABLISH COMMUNICATIONS WITH SERIAL DEVICE"
            self.tab8.ElogView.insert(ctk.END, error_message)
            
            # Save error log
            value = self.tab8.ElogView.get(0, ctk.END)
            with open("ErrorLog", "wb") as f:
                pickle.dump(value, f)
            return  # Exit if connection fails

        numChar = int(self.com3charPortEntryField.get())
        response = self.ser3.read(numChar).strip().decode('utf-8')
        
        # Update output field
        self.com3outPortEntryField.delete(0, ctk.END)
        self.com3outPortEntryField.insert(0, response)

    def Servo(self):
        try:
            selRow = self.tab1.progView.curselection()[0]
            selRow += 1
        except IndexError:
            last = self.tab1.progView.size() - 1
            selRow = last
            self.tab1.progView.select_set(selRow)

        servoNum = self.servoNumEntryField.get()
        servoPos = self.servoPosEntryField.get()
        servoins = f"Servo number {servoNum} to position: {servoPos}"
        
        self.tab1.progView.insert(selRow, bytes(servoins + '\n', 'utf-8'))
        self.tab1.progView.selection_clear(0, ctk.END)
        self.tab1.progView.select_set(selRow)
        
        items = self.tab1.progView.get(0, ctk.END)
        file_path = os.path.relpath(self.ProgEntryField.get())
        
        with open(file_path, 'w', encoding='utf-8') as f:
            for item in items:
                f.write(item.strip().decode('utf-8'))
                f.write('\n')

    def loadProg(self):
        # Determine the folder based on whether the app is frozen (e.g., compiled with PyInstaller) or running as a script
        if getattr(sys, 'frozen', False):
            folder = os.path.dirname(sys.executable)
        else:
            folder = os.path.dirname(os.path.realpath(__file__))

        filetypes = (('Robot Program', '*.ar'), ("All Files", "*.*"))
        filename = fd.askopenfilename(title='Open File', initialdir=folder, filetypes=filetypes)
        
        if filename:
            name = os.path.basename(filename)
            self.ProgEntryField.delete(0, ctk.END)
            self.ProgEntryField.insert(0, name)
            
            # Clear the current content in progView and load the selected file
            self.tab1.progView.delete(0, ctk.END)
            
            with open(filename, "rb") as Prog:
                time.sleep(0.1)  # Optional sleep
                for item in Prog:
                    self.tab1.progView.insert(ctk.END, item)
            
            self.tab1.progView.pack()
            self.scrollbar.configure(command=self.tab1.progView.yview)
            self.savePosData()

    def callProg(self, name):
        # Update the program entry field with the provided name
        self.ProgEntryField.delete(0, 'end')
        self.ProgEntryField.insert(0, name)
        
        # Clear the current content in progView
        self.tab1.progView.delete(0, 'end')
        
        # Open the file in text mode and insert each line into progView
        with open(name, "r") as Prog:
            time.sleep(0.1)  # Optional delay
            for item in Prog:
                self.tab1.progView.insert('end', item.rstrip('\n'))
        
        # Configure scrollbar for the text widget
        self.scrollbar.configure(command=self.tab1.progView.yview)
        self.tab1.progView.configure(yscrollcommand=self.scrollbar.set)

    def CreateProg(self):
        # Prompt user for a new program name using CustomTkinter's simpledialog equivalent
        user_input = self.simpledialog.askstring(title="New Program", prompt="New Program Name:")
        if not user_input:
            return  # Exit if the user cancels or provides no input
        
        file_path = f"{user_input}.ar"
        
        # Create a new file and write initial content
        with open(file_path, 'w', encoding='utf-8') as f:
            f.write("##BEGINNING OF PROGRAM##\n")
        
        # Update the program entry field with the new file path
        self.ProgEntryField.delete(0, 'end')
        self.ProgEntryField.insert(0, file_path)
        
        # Clear the current content in progView and load the newly created file
        self.tab1.progView.delete(0, 'end')
        
        with open(file_path, "r") as Prog:
            time.sleep(0.1)  # Optional delay
            for item in Prog:
                self.tab1.progView.insert('end', item.rstrip('\n'))
        
        # Configure scrollbar for the text widget
        self.scrollbar.configure(command=self.tab1.progView.yview)
        self.tab1.progView.configure(yscrollcommand=self.scrollbar.set)
        
        self.savePosData()

    def insertCallProg(self):
        try:
            selRow = self.tab1.progView.curselection()[0]
            selRow += 1
        except IndexError:
            last = self.tab1.progView.size() - 1
            selRow = last
            self.tab1.progView.select_set(selRow)

        newProg = self.changeProgEntryField.get()
        changeProg = f"Call Program - {newProg}"
        
        # Ensure the program name has the correct extension
        if not changeProg.endswith(".ar"):
            changeProg += ".ar"
        
        # Insert the call program instruction
        self.tab1.progView.insert(selRow, bytes(changeProg + '\n', 'utf-8'))
        self.tab1.progView.selection_clear(0, ctk.END)
        self.tab1.progView.select_set(selRow)

        # Retrieve all items and save to file
        items = self.tab1.progView.get(0, ctk.END)
        file_path = os.path.relpath(self.ProgEntryField.get())
        
        with open(file_path, 'w', encoding='utf-8') as f:
            for item in items:
                f.write(item.strip().decode('utf-8'))
                f.write('\n')

    def insertGCprog(self):
        try:
            selRow = self.tab1.progView.curselection()[0]
            selRow += 1
        except IndexError:
            last = self.tab1.progView.size() - 1
            selRow = last
            self.tab1.progView.select_set(selRow)

        newProg = self.PlayGCEntryField.get()
        GCProg = f"Run Gcode Program - {newProg}"
        
        # Insert the Gcode program instruction
        self.tab1.progView.insert(selRow, bytes(GCProg + '\n', 'utf-8'))
        self.tab1.progView.selection_clear(0, ctk.END)
        self.tab1.progView.select_set(selRow)

        # Retrieve all items and save to file
        items = self.tab1.progView.get(0, ctk.END)
        file_path = os.path.relpath(self.ProgEntryField.get())
        
        with open(file_path, 'w', encoding='utf-8') as f:
            for item in items:
                f.write(item.strip().decode('utf-8'))
                f.write('\n')

    def insertReturn(self):
        try:
            selRow = self.tab1.progView.curselection()[0]
            selRow += 1
        except IndexError:
            last = self.tab1.progView.size() - 1
            selRow = last
            self.tab1.progView.select_set(selRow)

        value = "Return"
        
        # Insert the return instruction
        self.tab1.progView.insert(selRow, bytes(value + '\n', 'utf-8'))
        self.tab1.progView.selection_clear(0, ctk.END)
        self.tab1.progView.select_set(selRow)

        # Retrieve all items and save to file
        items = self.tab1.progView.get(0, ctk.END)
        file_path = os.path.relpath(self.ProgEntryField.get())
        
        with open(file_path, 'w', encoding='utf-8') as f:
            for item in items:
                f.write(item.strip().decode('utf-8'))
                f.write('\n')

    def openText(self):
        # Get the file path from the program entry field
        file_path = os.path.relpath(self.ProgEntryField.get())
        
        if os.path.exists(file_path):
            os.startfile(file_path)
        else:
            print(f"File not found: {file_path}")

    def reloadProg(self):
        file_path = os.path.relpath(self.ProgEntryField.get())
        
        # Update the program entry field with the reloaded file path
        self.ProgEntryField.delete(0, 'end')
        self.ProgEntryField.insert(0, file_path)
        
        # Clear the current content in progView and load the file
        self.tab1.progView.delete(0, 'end')
        
        with open(file_path, "r") as Prog:
            time.sleep(0.1)  # Optional delay for smoother loading
            for item in Prog:
                self.tab1.progView.insert('end', item.rstrip('\n'))
        
        # Configure scrollbar for the text widget
        self.scrollbar.configure(command=self.tab1.progView.yview)
        self.tab1.progView.configure(yscrollcommand=self.scrollbar.set)
        
        self.savePosData()

    def insertvisFind(self):        
        try:
            selRow = self.tab1.progView.curselection()[0]
            selRow += 1
        except IndexError:
            selRow = self.tab1.progView.size() - 1
            self.tab1.progView.select_set(selRow)

        # Get template and background color settings
        template = self.selectedTemplate.get() or "None_Selected.jpg"
        autoBGVal = int(self.autoBG.get())
        BGcolor = "(Auto)" if autoBGVal == 1 else self.VisBacColorEntryField.get()
        
        # Retrieve score, pass, and fail tab values
        score = self.VisScoreEntryField.get()
        passTab = self.visPassEntryField.get()
        failTab = self.visFailEntryField.get()
        
        # Construct the command string
        value = f"Vis Find - {template} - BGcolor {BGcolor} Score {score} Pass {passTab} Fail {failTab}"
        
        # Insert the command and update selection
        self.tab1.progView.insert(selRow, bytes(value + '\n', 'utf-8'))
        self.tab1.progView.selection_clear(0, ctk.END)
        self.tab1.progView.select_set(selRow)
        
        # Save all items to file
        items = self.tab1.progView.get(0, ctk.END)
        file_path = os.path.relpath(self.ProgEntryField.get())
        
        with open(file_path, 'w', encoding='utf-8') as f:
            for item in items:
                f.write(item.strip().decode('utf-8'))
                f.write('\n')

    def IfRegjumpTab(self):
        try:
            # Attempt to get the current selection and set the insertion row
            selRow = self.tab1.progView.curselection()[0]
            selRow += 1
        except IndexError:
            selRow = self.tab1.progView.size() - 1
            self.tab1.progView.select_set(selRow)
        
        # Get the register number, comparison value, and target tab
        regNum = self.regNumJmpEntryField.get()
        regEqNum = self.regEqJmpEntryField.get()
        tabNum = self.regTabJmpEntryField.get()
        
        # Construct the command string
        tabjmp = f"If Register {regNum} = {regEqNum} Jump to Tab {tabNum}"
        
        # Insert command into progView
        self.tab1.progView.insert(selRow, bytes(tabjmp + '\n', 'utf-8'))
        self.tab1.progView.selection_clear(0, ctk.END)
        self.tab1.progView.select_set(selRow)
        
        # Save all items in progView to file
        items = self.tab1.progView.get(0, ctk.END)
        file_path = os.path.relpath(self.ProgEntryField.get())
        
        with open(file_path, 'w', encoding='utf-8') as f:
            for item in items:
                f.write(item.strip().decode('utf-8'))
                f.write('\n')

    def insertRegister(self):
        try:
            # Attempt to get the current selection and set the insertion row
            selRow = self.tab1.progView.curselection()[0]
            selRow += 1
        except IndexError:
            selRow = self.tab1.progView.size() - 1
            self.tab1.progView.select_set(selRow)
        
        # Get register number and command
        regNum = self.regNumEntryField.get()
        regCmd = self.regEqEntryField.get()
        
        # Construct the register command string
        regIns = f"Register {regNum} = {regCmd}"
        
        # Insert command into progView
        self.tab1.progView.insert(selRow, bytes(regIns + '\n', 'utf-8'))
        self.tab1.progView.selection_clear(0, ctk.END)
        self.tab1.progView.select_set(selRow)
        
        # Save all items in progView to file
        items = self.tab1.progView.get(0, ctk.END)
        file_path = os.path.relpath(self.ProgEntryField.get())
        
        with open(file_path, 'w', encoding='utf-8') as f:
            for item in items:
                f.write(item.strip().decode('utf-8'))
                f.write('\n')

    def storPos(self):
        try:
            # Attempt to get the current selection and set the insertion row
            selRow = self.tab1.progView.curselection()[0]
            selRow += 1
        except IndexError:
            selRow = self.tab1.progView.size() - 1
            self.tab1.progView.select_set(selRow)
        
        # Retrieve values from entry fields
        regNum = self.storPosNumEntryField.get()
        regElmnt = self.storPosElEntryField.get()
        regCmd = self.storPosValEntryField.get()
        
        # Construct the position register command string
        regIns = f"Position Register {regNum} Element {regElmnt} = {regCmd}"
        
        # Insert the command into progView
        self.tab1.progView.insert(selRow, bytes(regIns + '\n', 'utf-8'))
        self.tab1.progView.selection_clear(0, ctk.END)
        self.tab1.progView.select_set(selRow)
        
        # Save all items in progView to file
        items = self.tab1.progView.get(0, ctk.END)
        file_path = os.path.relpath(self.ProgEntryField.get())
        
        with open(file_path, 'w', encoding='utf-8') as f:
            for item in items:
                f.write(item.strip().decode('utf-8'))
                f.write('\n')

    def insCalibrate(self):
        try:
            # Attempt to get the current selection and set the insertion row
            selRow = self.tab1.progView.curselection()[0] + 1
        except IndexError:
            # Default to the end if there is no selection
            selRow = self.tab1.progView.size() - 1
            self.tab1.progView.select_set(selRow)
        
        # Define the calibration command
        insCal = "Calibrate Robot"
        
        # Insert the command into progView
        self.tab1.progView.insert(selRow, bytes(insCal + '\n', 'utf-8'))
        self.tab1.progView.selection_clear(0, ctk.END)
        self.tab1.progView.select_set(selRow)
        
        # Save all items in progView to file
        items = self.tab1.progView.get(0, ctk.END)
        file_path = os.path.relpath(self.ProgEntryField.get())
        
        with open(file_path, 'w', encoding='utf-8') as f:
            for item in items:
                f.write(item.strip().decode('utf-8'))
                f.write('\n')

    def progViewselect(self, event):
        try:
            # Get the selected row index in progView
            selRow = self.tab1.progView.curselection()[0]
            
            # Update curRowEntryField with the selected row index
            self.curRowEntryField.delete(0, ctk.END)
            self.curRowEntryField.insert(0, selRow)
        except IndexError:
            # Handle case where no item is selected
            self.curRowEntryField.delete(0, ctk.END)

    def getSel(self):
        try:
            # Get the selected row index in progView
            selRow = self.tab1.progView.curselection()[0]
            
            # Scroll the view to make the selected row visible
            self.tab1.progView.see(selRow + 2)
            
            data = list(map(int, self.tab1.progView.curselection()))
            command = self.tab1.progView.get(data[0]).decode()

            self.manEntryField.delete(0, ctk.END)
            self.manEntryField.insert(0, command)
        except IndexError:
            # Handle case where no item is selected
            self.manEntryField.delete(0, ctk.END)

    def control_servo(self, servo_number, position_field):
        self.savePosData()
        servoPos = position_field.get()
        command = f"SV{servo_number}P{servoPos}\n"
        self.ser2.write(command.encode())
        self.ser2.flushInput()
        time.sleep(0.1)
        self.ser2.read()

    # Refactored servo control functions
    def Servo0on(self): self.control_servo(0, self.servo0onEntryField)
    def Servo0off(self): self.control_servo(0, self.servo0offEntryField)
    def Servo1on(self): self.control_servo(1, self.servo1onEntryField)
    def Servo1off(self): self.control_servo(1, self.servo1offEntryField)
    def Servo2on(self): self.control_servo(2, self.servo2onEntryField)
    def Servo2off(self): self.control_servo(2, self.servo2offEntryField)
    def Servo3on(self): self.control_servo(3, self.servo3onEntryField)
    def Servo3off(self): self.control_servo(3, self.servo3offEntryField)

    def control_output(self, action, output_field):
        outputNum = output_field.get()
        command = f"{action}X{outputNum}\n"
        self.ser2.write(command.encode())
        self.ser2.flushInput()
        time.sleep(0.1)
        self.ser2.read()

    # Refactored digital output control functions
    def DO1on(self): self.control_output("ON", self.DO1onEntryField)
    def DO1off(self): self.control_output("OF", self.DO1offEntryField)
    def DO2on(self): self.control_output("ON", self.DO2onEntryField)
    def DO2off(self): self.control_output("OF", self.DO2offEntryField)
    def DO3on(self): self.control_output("ON", self.DO3onEntryField)
    def DO3off(self): self.control_output("OF", self.DO3offEntryField)
    def DO4on(self): self.control_output("ON", self.DO4onEntryField)
    def DO4off(self): self.control_output("OF", self.DO4offEntryField)
    def DO5on(self): self.control_output("ON", self.DO5onEntryField)
    def DO5off(self): self.control_output("OF", self.DO5offEntryField)
    def DO6on(self): self.control_output("ON", self.DO6onEntryField)
    def DO6off(self): self.control_output("OF", self.DO6offEntryField)

    def TestString(self):
        # Construct command and send it
        command = "TM" + self.testSendEntryField.get() + "\n"
        self.ser.write(command.encode())
        self.ser.flushInput()
        
        # Read and display the response
        echo = self.ser.readline()
        self.testRecEntryField.delete(0, 'end')
        self.testRecEntryField.insert(0, echo)

    def ClearTestString(self):
        # Clear the test receive entry field
        self.testRecEntryField.delete(0, 'end')

    def CalcLinDist(self, X2, Y2, Z2):

        # Calculate the linear distance between the current position and (X2, Y2, Z2)
        self.LineDist = (((X2 - self.XcurPos) ** 2) + ((Y2 - self.YcurPos) ** 2) + ((Z2 - self.ZcurPos) ** 2)) ** 0.5
        return self.LineDist

    def CalcLinVect(self, X2, Y2, Z2):

        # Calculate the vector components from the current position to (X2, Y2, Z2)
        self.Xv = X2 - self.XcurPos
        self.Yv = Y2 - self.YcurPos
        self.Zv = Z2 - self.ZcurPos

        return self.Xv, self.Yv, self.Zv

    # Calibration and save defs #

    def calRobotAll(self):
        def create_calibration_command(stage_values, offsets):
            command = "LL" + "".join(
                f"{chr(65 + i)}{val}" for i, val in enumerate(stage_values + offsets)
            ) + "\n"
            return command

        def send_command(command):
            self.ser.write(command.encode())
            self.cmdSentEntryField.delete(0, 'end')
            self.cmdSentEntryField.insert(0, command)
            self.ser.flushInput()
            return str(self.ser.readline().strip(), 'utf-8')

        def handle_response(response, stage):
            success = response.startswith('A')
            self.displayPosition(response) if success else self.ErrorHandler(response)
            message = f"Auto Calibration Stage {stage} {'Successful' if success else 'Failed - See Log'}"
            style = "OK.TLabel" if success else "Alarm.TLabel"
            self.almStatusLab.configure(text=message, style=style)
            self.almStatusLab2.configure(text=message, style=style)
            return message

        def update_log(message):
            Curtime = datetime.datetime.now().strftime("%B %d %Y - %I:%M%p")
            self.tab8.ElogView.insert(END, f"{Curtime} - {message}")
            pickle.dump(self.tab8.ElogView.get(0, END), open("ErrorLog", "wb"))

        # Stage 1 Calibration
        stage1_values = [
            self.J1CalStat, self.J2CalStat, self.J3CalStat, self.J4CalStat, self.J5CalStat,
            self.J6CalStat, self.J7CalStat, self.J8CalStat, self.J9CalStat
        ]
        offsets = [
            self.J1calOff, self.J2calOff, self.J3calOff, self.J4calOff, self.J5calOff, self.J6calOff, self.J7calOff,
            self.J8calOff, self.J9calOff
        ]
        
        command = create_calibration_command(stage1_values, offsets)
        response = send_command(command)
        message = handle_response(response, stage=1)
        update_log(message)

        # Stage 2 Calibration
        stage2_values = [
            self.J1CalStat2, self.J2CalStat2, self.J3CalStat2, self.J4CalStat2, self.J5CalStat2,
            self.J6CalStat2, self.J7CalStat2, self.J8CalStat2, self.J9CalStat2
        ]
        
        if sum(stage2_values) > 0:
            command = create_calibration_command(stage2_values, offsets)
            response = send_command(command)
            message = handle_response(response, stage=2)
            update_log(message)

    def calibrate_joint(self, joint_id, joint_command):
        command = f"LL{joint_command}" + "J" + str(self.J1calOff) + "K" + str(self.J2calOff) + "L" + str(self.J3calOff) + "M" + str(
            self.J4calOff) + "N" + str(self.J5calOff) + "O" + str(self.J6calOff) + "P" + str(self.J7calOff) + "Q" + str(self.J8calOff) + "R" + str(self.J9calOff) + "\n"
        self.ser.write(command.encode())
        self.cmdSentEntryField.delete(0, 'end')
        self.cmdSentEntryField.insert(0, command)
        self.ser.flushInput()
        response = str(self.ser.readline().strip(), 'utf-8')
        self.cmdRecEntryField.delete(0, 'end')
        self.cmdRecEntryField.insert(0, response)
        
        if response.startswith("A"):
            self.displayPosition(response)
            message = f"J{joint_id} Calibrated Successfully"
            self.almStatusLab.configure(text=message, style="OK.TLabel")
            self.almStatusLab2.configure(text=message, style="OK.TLabel")
        else:
            message = f"J{joint_id} Calibration Failed"
            self.almStatusLab.configure(text=message, style="Alarm.TLabel")
            self.almStatusLab2.configure(text=message, style="Alarm.TLabel")
            self.ErrorHandler(response)
        
        Curtime = datetime.datetime.now().strftime("%B %d %Y - %I:%M%p")
        self.tab8.ElogView.insert(END, Curtime + " - " + message)
        value = self.tab8.ElogView.get(0, END)
        pickle.dump(value, open("ErrorLog", "wb"))

    # Refactored calibration functions for each joint
    def calRobotJ1(self): self.calibrate_joint(1, "A1B0C0D0E0F0G0H0I0")
    def calRobotJ2(self): self.calibrate_joint(2, "A0B1C0D0E0F0G0H0I0")
    def calRobotJ3(self): self.calibrate_joint(3, "A0B0C1D0E0F0G0H0I0")
    def calRobotJ4(self): self.calibrate_joint(4, "A0B0C0D1E0F0G0H0I0")
    def calRobotJ5(self): self.calibrate_joint(5, "A0B0C0D0E1F0G0H0I0")
    def calRobotJ6(self): self.calibrate_joint(6, "A0B0C0D0E0F1G0H0I0")
    def calRobotJ7(self): self.calibrate_joint(7, "A0B0C0D0E0F0G1H0I0")
    def calRobotJ8(self): self.calibrate_joint(8, "A0B0C0D0E0F0G0H1I0")
    def calRobotJ9(self): self.calibrate_joint(9, "A0B0C0D0E0F0G0H0I1")

    def calRobotMid():
        print("foo")
        # add command here

    def correctPos(self):
        def send_command(command):
            self.ser.write(command.encode())
            self.ser.flushInput()
            time.sleep(0.1)
            return str(self.ser.readline().strip(), 'utf-8')

        command = "CP\n"
        response = send_command(command)
        self.displayPosition(response)

    def requestPos(self):
        def send_command(command):
            self.ser.write(command.encode())
            self.ser.flushInput()
            time.sleep(0.1)
            return str(self.ser.readline().strip(), 'utf-8')

        command = "RP\n"
        response = send_command(command)
        self.displayPosition(response)

    def updateParams(self):
        def get_entry_fields():
            params = {
                "TFx": self.TFxEntryField.get(),
                "TFy": self.TFyEntryField.get(),
                "TFz": self.TFzEntryField.get(),
                "TFrz": self.TFrzEntryField.get(),
                "TFry": self.TFryEntryField.get(),
                "TFrx": self.TFrxEntryField.get(),
                "motDir": [self.J1MotDirEntryField.get(), self.J2MotDirEntryField.get(), self.J3MotDirEntryField.get(), self.J4MotDirEntryField.get(),
                        self.J5MotDirEntryField.get(), self.J6MotDirEntryField.get(), self.J7MotDirEntryField.get(), self.J8MotDirEntryField.get(), self.J9MotDirEntryField.get()],
                "calDir": [self.J1CalDirEntryField.get(), self.J2CalDirEntryField.get(), self.J3CalDirEntryField.get(), self.J4CalDirEntryField.get(),
                        self.J5CalDirEntryField.get(), self.J6CalDirEntryField.get(), self.J7CalDirEntryField.get(), self.J8CalDirEntryField.get(), self.J9CalDirEntryField.get()],
                "posLim": [self.J1PosLimEntryField.get(), self.J2PosLimEntryField.get(), self.J3PosLimEntryField.get(), self.J4PosLimEntryField.get(),
                        self.J5PosLimEntryField.get(), self.J6PosLimEntryField.get()],
                "negLim": [self.J1NegLimEntryField.get(), self.J2NegLimEntryField.get(), self.J3NegLimEntryField.get(), self.J4NegLimEntryField.get(),
                        self.J5NegLimEntryField.get(), self.J6NegLimEntryField.get()],
                "stepDeg": [self.J1StepDegEntryField.get(), self.J2StepDegEntryField.get(), self.J3StepDegEntryField.get(),
                            self.J4StepDegEntryField.get(), self.J5StepDegEntryField.get(), self.J6StepDegEntryField.get()],
                "encMult": [str(float(self.J1EncCPREntryField.get()) / float(self.J1DriveMSEntryField.get())),
                            str(float(self.J2EncCPREntryField.get()) / float(self.J2DriveMSEntryField.get())),
                            str(float(self.J3EncCPREntryField.get()) / float(self.J3DriveMSEntryField.get())),
                            str(float(self.J4EncCPREntryField.get()) / float(self.J4DriveMSEntryField.get())),
                            str(float(self.J5EncCPREntryField.get()) / float(self.J5DriveMSEntryField.get())),
                            str(float(self.J6EncCPREntryField.get()) / float(self.J6DriveMSEntryField.get()))],
                "dhTheta": [self.J1ΘEntryField.get(), self.J2ΘEntryField.get(), self.J3ΘEntryField.get(), self.J4ΘEntryField.get(),
                            self.J5ΘEntryField.get(), self.J6ΘEntryField.get()],
                "dhAlpha": [self.J1αEntryField.get(), self.J2αEntryField.get(), self.J3αEntryField.get(), self.J4αEntryField.get(),
                            self.J5αEntryField.get(), self.J6αEntryField.get()],
                "dhDist": [self.J1dEntryField.get(), self.J2dEntryField.get(), self.J3dEntryField.get(), self.J4dEntryField.get(),
                        self.J5dEntryField.get(), self.J6dEntryField.get()],
                "dhLink": [self.J1aEntryField.get(), self.J2aEntryField.get(), self.J3aEntryField.get(), self.J4aEntryField.get(),
                        self.J5aEntryField.get(), self.J6aEntryField.get()]
            }
            return params

        def configure_limits(self, params):
            limits = zip(
                [self.J1negLimLab, self.J2negLimLab, self.J3negLimLab, self.J4negLimLab, self.J5negLimLab, self.J6negLimLab],
                [self.J1posLimLab, self.J2posLimLab, self.J3posLimLab, self.J4posLimLab, self.J5posLimLab, self.J6posLimLab],
                [self.J1jogslide, self.J2jogslide, self.J3jogslide, self.J4jogslide, self.J5jogslide, self.J6jogslide],
                params["negLim"],
                params["posLim"]
            )
            for negLab, posLab, slide, negLim, posLim in limits:
                negLab.configure(text="-" + negLim)
                posLab.configure(text=posLim)
                slide.configure(
                    from_=-float(negLim), 
                    to=float(posLim), 
                    width=180,
                    orientation='horizontal'
                )

        def construct_command(params):
            command = (
                "UP" +
                f"A{params['TFx']}B{params['TFy']}C{params['TFz']}D{params['TFrz']}E{params['TFry']}F{params['TFrx']}" +
                "".join(f"{chr(71+i)}{motDir}" for i, motDir in enumerate(params['motDir'])) +
                "".join(f"{chr(80+i)}{calDir}" for i, calDir in enumerate(params['calDir'])) +
                "".join(f"{chr(89+i*2)}{posLim}{chr(90+i*2)}{negLim}" for i, (posLim, negLim) in enumerate(zip(params['posLim'], params['negLim']))) +
                "".join(f"{chr(107+i)}{stepDeg}" for i, stepDeg in enumerate(params['stepDeg'])) +
                "".join(f"{chr(113+i)}{encMult}" for i, encMult in enumerate(params['encMult'])) +
                "".join(f"{chr(119+i)}{dhTheta}" for i, dhTheta in enumerate(params['dhTheta'])) +
                "".join(f"{chr(35+i)}{dhAlpha}" for i, dhAlpha in enumerate(params['dhAlpha'])) +
                "".join(f"{chr(40+i)}{dhDist}" for i, dhDist in enumerate(params['dhDist'])) +
                "".join(f"{chr(60+i)}{dhLink}" for i, dhLink in enumerate(params['dhLink'])) +
                "\n"
            )
            return command

        params = get_entry_fields()
        configure_limits(params)
        command = construct_command(params)

        self.ser.write(command.encode())
        self.ser.flush()
        time.sleep(0.1)
        self.ser.flushInput()
        time.sleep(0.1)
        response = self.ser.read_all()

    def calExtAxis(self):
        def configure_axis(index, pos_limit, neg_limit_label, pos_limit_label, jog_slider, update_command):
            neg_limit = 0  # Constant for all axes in this context
            neg_limit_label.configure(text=str(-neg_limit))
            pos_limit_label.configure(text=str(pos_limit))
            jog_slider.configure(
                from_=-neg_limit, 
                to=pos_limit, 
                width=125, 
                orientation='horizontal', 
                command=update_command
            )

        # Retrieve and configure limits
        pos_limits = [
            float(self.axis7lengthEntryField.get()),
            float(self.axis8lengthEntryField.get()),
            float(self.axis9lengthEntryField.get())
        ]

        # Configure each axis
        configure_axis(7, pos_limits[0], self.J7negLimLab, self.J7posLimLab, self.J7jogslide, self.J7sliderUpdate)
        configure_axis(8, pos_limits[1], self.J8negLimLab, self.J8posLimLab, self.J8jogslide, self.J8sliderUpdate)
        configure_axis(9, pos_limits[2], self.J9negLimLab, self.J9posLimLab, self.J9jogslide, self.J9sliderUpdate)

        # Build command string
        command = (
            f"CEA{pos_limits[0]}B{self.J7rotation}C{self.J7steps}"
            f"D{pos_limits[1]}E{self.J8rotation}F{self.J8steps}"
            f"G{pos_limits[2]}H{self.J9rotation}I{self.J9steps}\n"
        )

        # Send command
        self.ser.write(command.encode())
        self.ser.flushInput()
        time.sleep(0.1)
        response = self.ser.read()

    def zero_axis(self, axis_number, axis_name):
        command = f"Z{axis_number}\n"
        self.ser.write(command.encode())
        self.ser.flushInput()
        time.sleep(0.1)
        status_text = f"{axis_name} Calibration Forced to Zero"
        self.almStatusLab.configure(text=status_text, style="Warn.TLabel")
        self.almStatusLab2.configure(text=status_text, style="Warn.TLabel")
        message = f"{axis_name} Calibration Forced to Zero - this is for commissioning and testing - be careful!"
        curtime = datetime.datetime.now().strftime("%B %d %Y - %I:%M%p")
        self.tab8.ElogView.insert(END, f"{curtime} - {message}")
        value = self.tab8.ElogView.get(0, END)
        pickle.dump(value, open("ErrorLog", "wb"))
        response = str(self.ser.readline().strip(), 'utf-8')
        self.displayPosition(response)

    # Main functions calling the helper function with specific parameters
    def zeroAxis7(self): self.zero_axis(7, "J7")
    def zeroAxis8(self): self.zero_axis(8, "J8")
    def zeroAxis9(self): self.zero_axis(9, "J9")

    def sendPos(self):
        # Create the command string with formatted current positions
        current_positions = {
            "A": self.J1AngCur, "B": self.J2AngCur, "C": self.J3AngCur, "D": self.J4AngCur,
            "E": self.J5AngCur, "F": self.J6AngCur, "G": self.J7PosCur, "H": self.J8PosCur, "I": self.J9PosCur
        }
        command = "SP" + "".join(f"{key}{value}" for key, value in current_positions.items()) + "\n"
        
        # Send the command
        self.ser.write(command.encode())
        self.ser.flushInput()
        time.sleep(0.1)
        response = self.ser.read()

    def CalZeroPos(self):
        # Record the current time for logging
        current_time = datetime.datetime.now().strftime("%B %d %Y - %I:%M%p")

        # Send zero calibration command
        command = "SPA0B0C0D0E90F0\n"
        self.ser.write(command.encode())
        self.ser.flushInput()
        time.sleep(0.1)
        self.ser.read()

        # Request updated position and update status labels
        self.requestPos()
        status_message = "Calibration Forced to Home"
        self.almStatusLab.configure(text=status_message, style="Warn.TLabel")
        self.almStatusLab2.configure(text=status_message, style="Warn.TLabel")

        # Log the calibration event
        log_message = f"{current_time} - {status_message} - this is for commissioning and testing - be careful!"
        self.tab8.ElogView.insert(END, log_message)
        log_content = self.tab8.ElogView.get(0, END)
        pickle.dump(log_content, open("ErrorLog", "wb"))

    def CalRestPos(self):
        # Record the current time for logging
        current_time = datetime.datetime.now().strftime("%B %d %Y - %I:%M%p")

        # Send rest position calibration command
        command = "SPA0B0C-89D0E0F0\n"
        self.ser.write(command.encode())
        self.ser.flushInput()
        time.sleep(0.1)
        self.ser.read()

        # Request updated position and update status labels
        self.requestPos()
        status_message = "Calibration Forced to Vertical Rest Pos"
        self.almStatusLab.configure(text=status_message, style="Warn.TLabel")
        self.almStatusLab2.configure(text=status_message, style="Warn.TLabel")

        # Log the calibration event
        log_message = f"{current_time} - Calibration Forced to Vertical - this is for commissioning and testing - be careful!"
        self.tab8.ElogView.insert(END, log_message)
        log_content = self.tab8.ElogView.get(0, END)
        pickle.dump(log_content, open("ErrorLog", "wb"))

    def displayPosition(self, response):
        # Update received command in entry field
        self.cmdRecEntryField.delete(0, 'end')
        self.cmdRecEntryField.insert(0, response)

        # Parse angles and positions
        joint_indices = {key: response.find(key) for key in "ABCDEFGHIJKLMNORPQR"}
        parsed_data = {
            "J1AngCur": response[joint_indices["A"] + 1 : joint_indices["B"]].strip(),
            "J2AngCur": response[joint_indices["B"] + 1 : joint_indices["C"]].strip(),
            "J3AngCur": response[joint_indices["C"] + 1 : joint_indices["D"]].strip(),
            "J4AngCur": response[joint_indices["D"] + 1 : joint_indices["E"]].strip(),
            "J5AngCur": response[joint_indices["E"] + 1 : joint_indices["F"]].strip(),
            "J6AngCur": response[joint_indices["F"] + 1 : joint_indices["G"]].strip(),
            "XcurPos": response[joint_indices["G"] + 1 : joint_indices["H"]].strip(),
            "YcurPos": response[joint_indices["H"] + 1 : joint_indices["I"]].strip(),
            "ZcurPos": response[joint_indices["I"] + 1 : joint_indices["J"]].strip(),
            "RzcurPos": response[joint_indices["J"] + 1 : joint_indices["K"]].strip(),
            "RycurPos": response[joint_indices["K"] + 1 : joint_indices["L"]].strip(),
            "RxcurPos": response[joint_indices["L"] + 1 : joint_indices["M"]].strip(),
            "SpeedVioation": response[joint_indices["M"] + 1 : joint_indices["N"]].strip(),
            "Debug": response[joint_indices["N"] + 1 : joint_indices["O"]].strip(),
            "Flag": response[joint_indices["O"] + 1 : joint_indices["P"]].strip(),
            "J7PosCur": float(response[joint_indices["P"] + 1 : joint_indices["Q"]].strip()),
            "J8PosCur": float(response[joint_indices["Q"] + 1 : joint_indices["R"]].strip()),
            "J9PosCur": float(response[joint_indices["R"] + 1 :].strip())
        }

        # Assign parsed data
        for key, value in parsed_data.items():
            setattr(self, key, value)

        # Determine wrist configuration
        WC = "F" if float(parsed_data["J5AngCur"]) > 0 else "N"

        # Update GUI elements
        entry_fields = [
            (self.J1curAngEntryField, parsed_data["J1AngCur"]),
            (self.J2curAngEntryField, parsed_data["J2AngCur"]),
            (self.J3curAngEntryField, parsed_data["J3AngCur"]),
            (self.J4curAngEntryField, parsed_data["J4AngCur"]),
            (self.J5curAngEntryField, parsed_data["J5AngCur"]),
            (self.J6curAngEntryField, parsed_data["J6AngCur"]),
            (self.XcurEntryField, parsed_data["XcurPos"]),
            (self.YcurEntryField, parsed_data["YcurPos"]),
            (self.ZcurEntryField, parsed_data["ZcurPos"]),
            (self.RzcurEntryField, parsed_data["RzcurPos"]),
            (self.RycurEntryField, parsed_data["RycurPos"]),
            (self.RxcurEntryField, parsed_data["RxcurPos"]),
            (self.J7curAngEntryField, parsed_data["J7PosCur"]),
            (self.J8curAngEntryField, parsed_data["J8PosCur"]),
            (self.J9curAngEntryField, parsed_data["J9PosCur"]),
            (self.manEntryField, parsed_data["Debug"])
        ]
        for field, value in entry_fields:
            field.delete(0, 'end')
            field.insert(0, value)

        # Update sliders
        jog_sliders = [
            (self.J1jogslide, parsed_data["J1AngCur"]),
            (self.J2jogslide, parsed_data["J2AngCur"]),
            (self.J3jogslide, parsed_data["J3AngCur"]),
            (self.J4jogslide, parsed_data["J4AngCur"]),
            (self.J5jogslide, parsed_data["J5AngCur"]),
            (self.J6jogslide, parsed_data["J6AngCur"]),
            (self.J7jogslide, parsed_data["J7PosCur"]),
            (self.J8jogslide, parsed_data["J8PosCur"]),
            (self.J9jogslide, parsed_data["J9PosCur"])
        ]
        for slider, value in jog_sliders:
            slider.set(value)

        # Save position data and handle errors
        self.savePosData()
        if parsed_data["Flag"]:
            self.ErrorHandler(parsed_data["Flag"])
        if parsed_data["SpeedVioation"] == '1':
            current_time = datetime.datetime.now().strftime("%B %d %Y - %I:%M%p")
            message = "Max Speed Violation - Reduce Speed Setpoint or Travel Distance"
            self.tab8.ElogView.insert(END, f"{current_time} - {message}")
            pickle.dump(self.tab8.ElogView.get(0, END), open("ErrorLog", "wb"))
            self.almStatusLab.configure(text=message, style="Warn.TLabel")
            self.almStatusLab2.configure(text=message, style="Warn.TLabel")

    def ClearKinTabFields(self):
        # Define field groups for organized clearing
        motion_dir_fields = [
            self.J1MotDirEntryField, self.J2MotDirEntryField, self.J3MotDirEntryField, self.J4MotDirEntryField,
            self.J5MotDirEntryField, self.J6MotDirEntryField, self.J7MotDirEntryField, self.J8MotDirEntryField, self.J9MotDirEntryField
        ]
        
        calibration_dir_fields = [
            self.J1CalDirEntryField, self.J2CalDirEntryField, self.J3CalDirEntryField, self.J4CalDirEntryField,
            self.J5CalDirEntryField, self.J6CalDirEntryField, self.J7CalDirEntryField, self.J8CalDirEntryField, self.J9CalDirEntryField
        ]
        
        position_limit_fields = [
            self.J1PosLimEntryField, self.J1NegLimEntryField, self.J2PosLimEntryField, self.J2NegLimEntryField,
            self.J3PosLimEntryField, self.J3NegLimEntryField, self.J4PosLimEntryField, self.J4NegLimEntryField,
            self.J5PosLimEntryField, self.J5NegLimEntryField, self.J6PosLimEntryField, self.J6NegLimEntryField
        ]
        
        step_deg_fields = [
            self.J1StepDegEntryField, self.J2StepDegEntryField, self.J3StepDegEntryField,
            self.J4StepDegEntryField, self.J5StepDegEntryField, self.J6StepDegEntryField
        ]
        
        drive_ms_fields = [
            self.J1DriveMSEntryField, self.J2DriveMSEntryField, self.J3DriveMSEntryField,
            self.J4DriveMSEntryField, self.J5DriveMSEntryField, self.J6DriveMSEntryField
        ]
        
        encoder_cpr_fields = [
            self.J1EncCPREntryField, self.J2EncCPREntryField, self.J3EncCPREntryField,
            self.J4EncCPREntryField, self.J5EncCPREntryField, self.J6EncCPREntryField
        ]
        
        theta_fields = [
            self.J1ΘEntryField, self.J2ΘEntryField, self.J3ΘEntryField, self.J4ΘEntryField,
            self.J5ΘEntryField, self.J6ΘEntryField
        ]
        
        alpha_fields = [
            self.J1αEntryField, self.J2αEntryField, self.J3αEntryField, self.J4αEntryField,
            self.J5αEntryField, self.J6αEntryField
        ]
        
        d_fields = [
            self.J1dEntryField, self.J2dEntryField, self.J3dEntryField, self.J4dEntryField,
            self.J5dEntryField, self.J6dEntryField
        ]
        
        a_fields = [
            self.J1aEntryField, self.J2aEntryField, self.J3aEntryField, self.J4aEntryField,
            self.J5aEntryField, self.J6aEntryField
        ]
        
        # Clear all fields
        for field_group in [
            motion_dir_fields, calibration_dir_fields, position_limit_fields,
            step_deg_fields, drive_ms_fields, encoder_cpr_fields, theta_fields,
            alpha_fields, d_fields, a_fields
        ]:
            for field in field_group:
                field.delete(0, 'end')

    ## Profiles defs ##

    ## AR4 Mk3 ##
    def LoadAR4Mk3default(self):
        self.ClearKinTabFields()

        # Define default values for each entry field
        default_values = {
            # Motor directions
            self.J1MotDirEntryField: 0, self.J2MotDirEntryField: 1, self.J3MotDirEntryField: 1,
            self.J4MotDirEntryField: 1, self.J5MotDirEntryField: 1, self.J6MotDirEntryField: 1,
            self.J7MotDirEntryField: 1, self.J8MotDirEntryField: 1, self.J9MotDirEntryField: 1,
            
            # Calibration directions
            self.J1CalDirEntryField: 1, self.J2CalDirEntryField: 0, self.J3CalDirEntryField: 1,
            self.J4CalDirEntryField: 0, self.J5CalDirEntryField: 0, self.J6CalDirEntryField: 1,
            self.J7CalDirEntryField: 0, self.J8CalDirEntryField: 0, self.J9CalDirEntryField: 0,
            
            # Position limits
            self.J1PosLimEntryField: 170, self.J1NegLimEntryField: 170, self.J2PosLimEntryField: 90,
            self.J2NegLimEntryField: 42, self.J3PosLimEntryField: 52, self.J3NegLimEntryField: 89,
            self.J4PosLimEntryField: 180, self.J4NegLimEntryField: 180, self.J5PosLimEntryField: 105,
            self.J5NegLimEntryField: 105, self.J6PosLimEntryField: 180, self.J6NegLimEntryField: 180,
            
            # Steps per degree
            self.J1StepDegEntryField: 44.4444, self.J2StepDegEntryField: 55.5555,
            self.J3StepDegEntryField: 55.5555, self.J4StepDegEntryField: 49.7777,
            self.J5StepDegEntryField: 21.8602, self.J6StepDegEntryField: 22.2222,
            
            # Drive MS settings
            self.J1DriveMSEntryField: 400, self.J2DriveMSEntryField: 400, self.J3DriveMSEntryField: 400,
            self.J4DriveMSEntryField: 400, self.J5DriveMSEntryField: 800, self.J6DriveMSEntryField: 400,
            
            # Encoder CPR settings
            self.J1EncCPREntryField: 4000, self.J2EncCPREntryField: 4000, self.J3EncCPREntryField: 4000,
            self.J4EncCPREntryField: 4000, self.J5EncCPREntryField: 4000, self.J6EncCPREntryField: 4000,
            
            # Θ (Theta) angles
            self.J1ΘEntryField: 0, self.J2ΘEntryField: -90, self.J3ΘEntryField: 0,
            self.J4ΘEntryField: 0, self.J5ΘEntryField: 0, self.J6ΘEntryField: 180,
            
            # α (Alpha) angles
            self.J1αEntryField: 0, self.J2αEntryField: -90, self.J3αEntryField: 0,
            self.J4αEntryField: -90, self.J5αEntryField: 90, self.J6αEntryField: -90,
            
            # d distances
            self.J1dEntryField: 169.77, self.J2dEntryField: 0, self.J3dEntryField: 0,
            self.J4dEntryField: 222.63, self.J5dEntryField: 0, self.J6dEntryField: 41,
            
            # a distances
            self.J1aEntryField: 0, self.J2aEntryField: 64.2, self.J3aEntryField: 305,
            self.J4aEntryField: 0, self.J5aEntryField: 0, self.J6aEntryField: 0
        }

        # Insert default values into each entry field
        for entry_field, value in default_values.items():
            entry_field.insert(0, str(value))

    def SaveAndApplyCalibration(self):
        
        # Set values from GUI inputs
        self.J7PosCur = self.J7curAngEntryField.get()
        self.J8PosCur = self.J8curAngEntryField.get()
        self.J9PosCur = self.J9curAngEntryField.get()
        self.VisProg = self.visoptions.get()
        self.J1calOff = float(self.J1calOffEntryField.get())
        self.J2calOff = float(self.J2calOffEntryField.get())
        self.J3calOff = float(self.J3calOffEntryField.get())
        self.J4calOff = float(self.J4calOffEntryField.get())
        self.J5calOff = float(self.J5calOffEntryField.get())
        self.J6calOff = float(self.J6calOffEntryField.get())
        self.J7calOff = float(self.J7calOffEntryField.get())
        self.J8calOff = float(self.J8calOffEntryField.get())
        self.J9calOff = float(self.J9calOffEntryField.get())
        self.J1OpenLoopVal = int(self.J1OpenLoopStat.get())
        self.J2OpenLoopVal = int(self.J2OpenLoopStat.get())
        self.J3OpenLoopVal = int(self.J3OpenLoopStat.get())
        self.J4OpenLoopVal = int(self.J4OpenLoopStat.get())
        self.J5OpenLoopVal = int(self.J5OpenLoopStat.get())
        self.J6OpenLoopVal = int(self.J6OpenLoopStat.get())
        self.DisableWristRotVal = int(self.DisableWristRot.get())
        self.J1CalStatVal = int(self.J1CalStat.get())
        self.J2CalStatVal = int(self.J2CalStat.get())
        self.J3CalStatVal = int(self.J3CalStat.get())
        self.J4CalStatVal = int(self.J4CalStat.get())
        self.J5CalStatVal = int(self.J5CalStat.get())
        self.J6CalStatVal = int(self.J6CalStat.get())
        self.J7CalStatVal = int(self.J7CalStat.get())
        self.J8CalStatVal = int(self.J8CalStat.get())
        self.J9CalStatVal = int(self.J9CalStat.get())
        self.J1CalStatVal2 = int(self.J1CalStat2.get())
        self.J2CalStatVal2 = int(self.J2CalStat2.get())
        self.J3CalStatVal2 = int(self.J3CalStat2.get())
        self.J4CalStatVal2 = int(self.J4CalStat2.get())
        self.J5CalStatVal2 = int(self.J5CalStat2.get())
        self.J6CalStatVal2 = int(self.J6CalStat2.get())
        self.J7CalStatVal2 = int(self.J7CalStat2.get())
        self.J8CalStatVal2 = int(self.J8CalStat2.get())
        self.J9CalStatVal2 = int(self.J9CalStat2.get())
        self.J7PosLim = float(self.axis7lengthEntryField.get())
        self.J7rotation = float(self.axis7rotEntryField.get())
        self.J7steps = float(self.axis7stepsEntryField.get())
        self.J8length = float(self.axis8lengthEntryField.get())
        self.J8rotation = float(self.axis8rotEntryField.get())
        self.J8steps = float(self.axis8stepsEntryField.get())
        self.J9length = float(self.axis9lengthEntryField.get())
        self.J9rotation = float(self.axis9rotEntryField.get())
        self.J9steps = float(self.axis9stepsEntryField.get())

        # Apply the updates and save
        try:
            self.updateParams()
            time.sleep(0.1)
            self.calExtAxis()
        except:
            print("No serial connection with Teensy board")

        self.savePosData()

    def savePosData(self):
        # Clear the calibration list and insert values sequentially
        self.calibration.delete(0, END)
        
        # Joint Angles
        self.calibration.insert(END, self.J1AngCur)
        self.calibration.insert(END, self.J2AngCur)
        self.calibration.insert(END, self.J3AngCur)
        self.calibration.insert(END, self.J4AngCur)
        self.calibration.insert(END, self.J5AngCur)
        self.calibration.insert(END, self.J6AngCur)

        # Current Positions (X, Y, Z, Rz, Ry, Rx)
        self.calibration.insert(END, self.XcurPos)
        self.calibration.insert(END, self.YcurPos)
        self.calibration.insert(END, self.ZcurPos)
        self.calibration.insert(END, self.RzcurPos)
        self.calibration.insert(END, self.RycurPos)
        self.calibration.insert(END, self.RxcurPos)

        # Ports and Program Entry Fields
        self.calibration.insert(END, self.comPortEntryField.get())
        self.calibration.insert(END, self.ProgEntryField.get())
        self.calibration.insert(END, self.servo0onEntryField.get())
        self.calibration.insert(END, self.servo0offEntryField.get())
        self.calibration.insert(END, self.servo1onEntryField.get())
        self.calibration.insert(END, self.servo1offEntryField.get())
        self.calibration.insert(END, self.DO1onEntryField.get())
        self.calibration.insert(END, self.DO1offEntryField.get())
        self.calibration.insert(END, self.DO2onEntryField.get())
        self.calibration.insert(END, self.DO2offEntryField.get())

        # Transform Fields (TFx to TFrz)
        self.calibration.insert(END, self.TFxEntryField.get())
        self.calibration.insert(END, self.TFyEntryField.get())
        self.calibration.insert(END, self.TFzEntryField.get())
        self.calibration.insert(END, self.TFrxEntryField.get())
        self.calibration.insert(END, self.TFryEntryField.get())
        self.calibration.insert(END, self.TFrzEntryField.get())

        # Joint 7 to 9 Calibration Fields
        self.calibration.insert(END, self.J7curAngEntryField.get())
        self.calibration.insert(END, self.J8curAngEntryField.get())
        self.calibration.insert(END, self.J9curAngEntryField.get())

        # Visual Calibration Fields
        self.calibration.insert(END, "VisFileLocEntryField")  # Placeholder
        self.calibration.insert(END, self.visoptions.get())
        self.calibration.insert(END, "VisPicOxPEntryField")
        self.calibration.insert(END, "VisPicOxMEntryField")
        self.calibration.insert(END, "VisPicOyPEntryField")
        self.calibration.insert(END, "VisPicOyMEntryField")
        self.calibration.insert(END, "VisPicXPEntryField")
        self.calibration.insert(END, "VisPicXMEntryField")
        self.calibration.insert(END, "VisPicYPEntryField")
        self.calibration.insert(END, "VisPicYMEntryField")

        # Calibration Offsets (J1 to J6)
        self.calibration.insert(END, self.J1calOffEntryField.get())
        self.calibration.insert(END, self.J2calOffEntryField.get())
        self.calibration.insert(END, self.J3calOffEntryField.get())
        self.calibration.insert(END, self.J4calOffEntryField.get())
        self.calibration.insert(END, self.J5calOffEntryField.get())
        self.calibration.insert(END, self.J6calOffEntryField.get())

        # Open Loop Values (J1 to J6)
        self.calibration.insert(END, self.J1OpenLoopVal)
        self.calibration.insert(END, self.J2OpenLoopVal)
        self.calibration.insert(END, self.J3OpenLoopVal)
        self.calibration.insert(END, self.J4OpenLoopVal)
        self.calibration.insert(END, self.J5OpenLoopVal)
        self.calibration.insert(END, self.J6OpenLoopVal)

        # Additional Configuration Fields
        self.calibration.insert(END, self.com2PortEntryField.get())
        self.calibration.insert(END, self.curTheme)
        self.calibration.insert(END, self.J1CalStatVal)
        self.calibration.insert(END, self.J2CalStatVal)
        self.calibration.insert(END, self.J3CalStatVal)
        self.calibration.insert(END, self.J4CalStatVal)
        self.calibration.insert(END, self.J5CalStatVal)
        self.calibration.insert(END, self.J6CalStatVal)

        # Joint 7 Calibration Parameters
        self.calibration.insert(END, self.J7PosLim)
        self.calibration.insert(END, self.J7rotation)
        self.calibration.insert(END, self.J7steps)
        self.calibration.insert(END, self.J7StepCur)

        # Joint Calibration Status Values (2nd Set)
        self.calibration.insert(END, self.J1CalStatVal2)
        self.calibration.insert(END, self.J2CalStatVal2)
        self.calibration.insert(END, self.J3CalStatVal2)
        self.calibration.insert(END, self.J4CalStatVal2)
        self.calibration.insert(END, self.J5CalStatVal2)
        self.calibration.insert(END, self.J6CalStatVal2)

        # Visual Settings
        self.calibration.insert(END, self.VisBrightSlide.get())
        self.calibration.insert(END, self.VisContrastSlide.get())
        self.calibration.insert(END, self.VisBacColorEntryField.get())
        self.calibration.insert(END, self.VisScoreEntryField.get())
        self.calibration.insert(END, self.VisX1PixEntryField.get())
        self.calibration.insert(END, self.VisY1PixEntryField.get())
        self.calibration.insert(END, self.VisX2PixEntryField.get())
        self.calibration.insert(END, self.VisY2PixEntryField.get())
        self.calibration.insert(END, self.VisX1RobEntryField.get())
        self.calibration.insert(END, self.VisY1RobEntryField.get())
        self.calibration.insert(END, self.VisX2RobEntryField.get())
        self.calibration.insert(END, self.VisY2RobEntryField.get())
        self.calibration.insert(END, self.VisZoomSlide.get())

        # Other Options
        self.calibration.insert(END, self.pick180.get())
        self.calibration.insert(END, self.pickClosest.get())
        self.calibration.insert(END, self.visoptions.get())
        self.calibration.insert(END, self.fullRot.get())
        self.calibration.insert(END, self.autoBG.get())

        # Miscellaneous Parameters
        self.calibration.insert(END, self.mX1)
        self.calibration.insert(END, self.mY1)
        self.calibration.insert(END, self.mX2)
        self.calibration.insert(END, self.mY2)

        # Joint 8 and 9 Parameters
        self.calibration.insert(END, self.J8length)
        self.calibration.insert(END, self.J8rotation)
        self.calibration.insert(END, self.J8steps)
        self.calibration.insert(END, self.J9length)
        self.calibration.insert(END, self.J9rotation)
        self.calibration.insert(END, self.J9steps)

        # Joint Calibration Offsets (J7 to J9)
        self.calibration.insert(END, self.J7calOffEntryField.get())
        self.calibration.insert(END, self.J8calOffEntryField.get())
        self.calibration.insert(END, self.J9calOffEntryField.get())

        # General Calibration Settings (GC_ST)
        self.calibration.insert(END, self.GC_ST_E1_EntryField.get())
        self.calibration.insert(END, self.GC_ST_E2_EntryField.get())
        self.calibration.insert(END, self.GC_ST_E3_EntryField.get())
        self.calibration.insert(END, self.GC_ST_E4_EntryField.get())
        self.calibration.insert(END, self.GC_ST_E5_EntryField.get())
        self.calibration.insert(END, self.GC_ST_E6_EntryField.get())
        self.calibration.insert(END, self.GC_SToff_E1_EntryField.get())
        self.calibration.insert(END, self.GC_SToff_E2_EntryField.get())
        self.calibration.insert(END, self.GC_SToff_E3_EntryField.get())
        self.calibration.insert(END, self.GC_SToff_E4_EntryField.get())
        self.calibration.insert(END, self.GC_SToff_E5_EntryField.get())
        self.calibration.insert(END, self.GC_SToff_E6_EntryField.get())

        # Wrist Rotation Disable
        self.calibration.insert(END, self.DisableWristRotVal)

        # Motor Direction Fields (J1 to J9)
        self.calibration.insert(END, self.J1MotDirEntryField.get())
        self.calibration.insert(END, self.J2MotDirEntryField.get())
        self.calibration.insert(END, self.J3MotDirEntryField.get())
        self.calibration.insert(END, self.J4MotDirEntryField.get())
        self.calibration.insert(END, self.J5MotDirEntryField.get())
        self.calibration.insert(END, self.J6MotDirEntryField.get())
        self.calibration.insert(END, self.J7MotDirEntryField.get())
        self.calibration.insert(END, self.J8MotDirEntryField.get())
        self.calibration.insert(END, self.J9MotDirEntryField.get())

        # Calibration Direction Fields (J1 to J9)
        self.calibration.insert(END, self.J1CalDirEntryField.get())
        self.calibration.insert(END, self.J2CalDirEntryField.get())
        self.calibration.insert(END, self.J3CalDirEntryField.get())
        self.calibration.insert(END, self.J4CalDirEntryField.get())
        self.calibration.insert(END, self.J5CalDirEntryField.get())
        self.calibration.insert(END, self.J6CalDirEntryField.get())
        self.calibration.insert(END, self.J7CalDirEntryField.get())
        self.calibration.insert(END, self.J8CalDirEntryField.get())
        self.calibration.insert(END, self.J9CalDirEntryField.get())

        # Position Limits Fields (J1 to J9)
        self.calibration.insert(END, self.J1PosLimEntryField.get())
        self.calibration.insert(END, self.J1NegLimEntryField.get())
        self.calibration.insert(END, self.J2PosLimEntryField.get())
        self.calibration.insert(END, self.J2NegLimEntryField.get())
        self.calibration.insert(END, self.J3PosLimEntryField.get())
        self.calibration.insert(END, self.J3NegLimEntryField.get())
        self.calibration.insert(END, self.J4PosLimEntryField.get())
        self.calibration.insert(END, self.J4NegLimEntryField.get())
        self.calibration.insert(END, self.J5PosLimEntryField.get())
        self.calibration.insert(END, self.J5NegLimEntryField.get())
        self.calibration.insert(END, self.J6PosLimEntryField.get())
        self.calibration.insert(END, self.J6NegLimEntryField.get())
        self.calibration.insert(END, self.J7PosLimEntryField.get())
        self.calibration.insert(END, self.J7NegLimEntryField.get())
        self.calibration.insert(END, self.J8PosLimEntryField.get())
        self.calibration.insert(END, self.J8NegLimEntryField.get())
        self.calibration.insert(END, self.J9PosLimEntryField.get())
        self.calibration.insert(END, self.J9NegLimEntryField.get())

        # Encoder Settings (J1 to J9)
        self.calibration.insert(END, self.J1EncCPREntryField.get())
        self.calibration.insert(END, self.J2EncCPREntryField.get())
        self.calibration.insert(END, self.J3EncCPREntryField.get())
        self.calibration.insert(END, self.J4EncCPREntryField.get())
        self.calibration.insert(END, self.J5EncCPREntryField.get())
        self.calibration.insert(END, self.J6EncCPREntryField.get())

        # Drive Modes (J1 to J6)
        self.calibration.insert(END, self.J1DriveMSEntryField.get())
        self.calibration.insert(END, self.J2DriveMSEntryField.get())
        self.calibration.insert(END, self.J3DriveMSEntryField.get())
        self.calibration.insert(END, self.J4DriveMSEntryField.get())
        self.calibration.insert(END, self.J5DriveMSEntryField.get())
        self.calibration.insert(END, self.J6DriveMSEntryField.get())

        # Step Degrees (J1 to J6)
        self.calibration.insert(END, self.J1StepDegEntryField.get())
        self.calibration.insert(END, self.J2StepDegEntryField.get())
        self.calibration.insert(END, self.J3StepDegEntryField.get())
        self.calibration.insert(END, self.J4StepDegEntryField.get())
        self.calibration.insert(END, self.J5StepDegEntryField.get())
        self.calibration.insert(END, self.J6StepDegEntryField.get())
        
        # Serialize and save the data
        value = self.calibration.get(0, END)
        pickle.dump(value, open("ARbot.cal", "wb"))

    def checkSpeedVals(self):
        speedtype = self.speedOption.get()
        
        # Validate and update a field with a default value if out of bounds.
        def validate_and_update(field, value, min_val, max_val=None):
            speed = float(field.get())
            if speed <= min_val or (max_val is not None and speed > max_val):
                field.delete(0, 'end')
                field.insert(0, str(value))
        
        # Validate speed based on type
        if speedtype == "mm per Sec":
            validate_and_update(self.speedEntryField, 5, 0.01)
        elif speedtype == "Seconds":
            validate_and_update(self.speedEntryField, 1, 0.001)
        elif speedtype == "Percent":
            validate_and_update(self.speedEntryField, 10, 0.01, 100)
        
        # Validate acceleration, deceleration, and ramp fields
        validate_and_update(self.ACCspeedField, 10, 0.01, 100)
        validate_and_update(self.DECspeedField, 10, 0.01, 100)
        validate_and_update(self.ACCrampField, 50, 0.01, 100)

    def ErrorHandler(self, response):
        Curtime = datetime.datetime.now().strftime("%B %d %Y - %I:%M%p")
        
        # Log error message to ElogView and save to a file.
        def log_error(message):
            self.tab8.ElogView.insert(END, f"{Curtime} - {message}")
            pickle.dump(self.tab8.ElogView.get(0, END), open("ErrorLog", "wb"))

        # Update alarm labels with a specific message and style.
        def update_alarm_status(message):
            self.almStatusLab.configure(text=message, style="Alarm.TLabel")
            self.almStatusLab2.configure(text=message, style="Alarm.TLabel")
            self.GCalmStatusLab.configure(text=message, style="Alarm.TLabel")

        # Handle specific axis limit errors based on response.
        def handle_axis_limit_error():
            axis_errors = {
                2: "J1 Axis Limit", 3: "J2 Axis Limit", 4: "J3 Axis Limit",
                5: "J4 Axis Limit", 6: "J5 Axis Limit", 7: "J6 Axis Limit",
                8: "J7 Axis Limit", 9: "J8 Axis Limit", 10: "J9 Axis Limit"
            }
            for i, message in axis_errors.items():
                if response[i:i+1] == '1':
                    log_error(message)
            update_alarm_status("Axis Limit Error - See Log")

        # Handle specific collision or motor errors based on response.
        def handle_collision_error():
            collision_errors = {
                2: "J1 Collision or Motor Error", 3: "J2 Collision or Motor Error",
                4: "J3 Collision or Motor Error", 5: "J4 Collision or Motor Error",
                6: "J5 Collision or Motor Error", 7: "J6 Collision or Motor Error"
            }
            for i, message in collision_errors.items():
                if response[i:i+1] == '1':
                    log_error(message)
                    self.correctPos()
                    self.stopProg()
            update_alarm_status("Collision or Motor Error - See Log")

        # Handle specific calibration errors based on response.
        def handle_calibration_error():
            calibration_errors = {
                "1": "J1 CALIBRATION ERROR", "2": "J2 CALIBRATION ERROR", "3": "J3 CALIBRATION ERROR",
                "4": "J4 CALIBRATION ERROR", "5": "J5 CALIBRATION ERROR", "6": "J6 CALIBRATION ERROR",
                "7": "J7 CALIBRATION ERROR", "8": "J8 CALIBRATION ERROR", "9": "J9 CALIBRATION ERROR"
            }
            axis_id = response[2:3]
            if axis_id in calibration_errors:
                log_error(calibration_errors[axis_id])

        self.cmdRecEntryField.delete(0, 'end')
        self.cmdRecEntryField.insert(0, response)
        
        # Axis Limit Error
        if response[1:2] == 'L':
            handle_axis_limit_error()
            self.stopProg()

        # Collision Error
        elif response[1:2] == 'C':
            handle_collision_error()

        # Position Out of Reach
        elif response[1:2] == 'R':
            self.posOutreach = True
            self.stopProg()
            log_error("Position Out of Reach")
            update_alarm_status("Position Out of Reach")

        # Spline Error
        elif response[1:2] == 'S':
            self.stopProg()
            log_error("Spline Can Only Have Move L Types")
            update_alarm_status("Spline Can Only Have Move L Types")

        # GCode Error
        elif response[1:2] == 'G':
            self.stopProg()
            log_error("Gcode file not found")
            update_alarm_status("Gcode file not found")

        # Estop Button Pressed
        elif response[1:2] == 'B':
            self.estopActive = True
            self.stopProg()
            log_error("Estop Button was Pressed")
            update_alarm_status("Estop Button was Pressed")

        # Calibration Error
        elif response[1:2] == 'A':
            handle_calibration_error()

        # Unknown Error
        else:
            self.stopProg()
            log_error("Unknown Error")
            update_alarm_status("Unknown Error")

    # Vision defs #

    def testvis(self):
        visprog = self.visoptions.get()
        visprog_functions = {
            "Openvision": self.openvision,
            "Roborealm 1.7.5": self.roborealm175,
            "x,y,r": self.xyr
        }
        # Call the function if the visprog option exists in the mapping
        if visprog in visprog_functions:
            visprog_functions[visprog]()

    def openvision(self):
        self.visfail = 1

        # Update system status label
        def update_status(label, text="SYSTEM READY", style="OK.TLabel"):
            label.configure(text=text, style=style)

        # Read the last line from a file, retry if file not found.
        def read_last_line(file_path):
            while True:
                try:
                    with open(file_path, "r") as f:
                        return f.readlines()[-1]
                except FileNotFoundError:
                    time.sleep(0.1)

        # Update an entry field with a new value.
        def update_entry(field, value):
            field.delete(0, 'end')
            field.insert(0, value)

        # Main loop for vision processing
        while self.visfail:
            update_status(self.almStatusLab)
            update_status(self.almStatusLab2)
            value = read_last_line(self.VisFileLoc)

            x = int(value[110:122])
            y = int(value[130:142])
            self.viscalc(x, y)

            self.visfail = self.Ypos > self.VisEndYmm
            if self.visfail:
                time.sleep(0.1)

        open(self.VisFileLoc, "w").close()  # Clear the vision file

        # Update fields with position data
        update_entry(self.VisXfindEntryField, self.Xpos)
        update_entry(self.VisYfindEntryField, self.Ypos)
        update_entry(self.VisRZfindEntryField, 0)
        update_entry(self.VisXpixfindEntryField, x)
        update_entry(self.VisYpixfindEntryField, y)
        update_entry(self.SP_1_E1_EntryField, self.Xpos)
        update_entry(self.SP_1_E2_EntryField, self.Ypos)

    def roborealm175(self):
        self.visfail = 1

        # Update a label with specified text and style.
        def update_status(label, text, style):
            label.configure(text=text, style=style)

        # Read the last line from a file, retry if file not found.
        def read_last_line(file_path):
            while True:
                try:
                    with open(file_path, "r") as f:
                        return f.readlines()[-1]
                except FileNotFoundError:
                    time.sleep(0.1)

        # Update an entry field with a new value.
        def update_entry(field, value):
            field.delete(0, 'end')
            field.insert(0, value)

        # Main loop for processing vision data
        while self.visfail:
            update_status(self.almStatusLab, "WAITING FOR CAMERA", "Alarm.TLabel")
            update_status(self.almStatusLab2, "WAITING FOR CAMERA", "Alarm.TLabel")

            value = read_last_line(self.VisFileLoc)

            update_status(self.almStatusLab, "SYSTEM READY", "OK.TLabel")
            update_status(self.almStatusLab2, "SYSTEM READY", "OK.TLabel")

            # Extract x and y values from the comma-separated value
            index = value.index(',')
            x = float(value[:index])
            y = float(value[index+1:])
            
            self.viscalc(x, y)

            self.visfail = float(self.Ypos) > float(self.VisEndYmm)
            if self.visfail:
                time.sleep(0.1)

        open(self.VisFileLoc, "w").close()  # Clear the vision file

        # Update fields with position data
        update_entry(self.VisXfindEntryField, self.Xpos)
        update_entry(self.VisYfindEntryField, self.Ypos)
        update_entry(self.VisRZfindEntryField, 0)
        update_entry(self.VisXpixfindEntryField, x)
        update_entry(self.VisYpixfindEntryField, y)
        update_entry(self.SP_1_E1_EntryField, self.Xpos)
        update_entry(self.SP_1_E2_EntryField, self.Ypos)

    def xyr(self):
        self.visfail = 1

        # Update a label with specified text and style.
        def update_status(label, text, style):
            label.configure(text=text, style=style)

        # Read the last line from a file, retry if file not found.
        def read_last_line(file_path):
            while True:
                try:
                    with open(file_path, "r") as f:
                        return f.readlines()[-1]
                except FileNotFoundError:
                    time.sleep(0.1)

        # Update an entry field with a new value.
        def update_entry(field, value):
            field.delete(0, 'end')
            field.insert(0, value)

        # Main loop for processing vision data
        while self.visfail:
            update_status(self.almStatusLab, "SYSTEM READY", "OK.TLabel")
            update_status(self.almStatusLab2, "SYSTEM READY", "OK.TLabel")

            value = read_last_line(self.VisFileLoc)

            # Update status to indicate system is ready
            update_status(self.almStatusLab, "SYSTEM READY", "OK.TLabel")
            update_status(self.almStatusLab2, "SYSTEM READY", "OK.TLabel")

            # Parse x, y, r values from comma-separated string
            index1 = value.index(',')
            x = float(value[:index1])

            remaining_value = value[index1 + 1:]
            index2 = remaining_value.index(',')
            y = float(remaining_value[:index2])
            r = float(remaining_value[index2 + 1:])

            self.viscalc(x, y)

            self.visfail = self.Ypos > float(self.VisEndYmm)
            if self.visfail:
                time.sleep(0.1)

        open(self.VisFileLoc, "w").close()  # Clear the vision file

        # Update fields with position and rotation data
        update_entry(self.VisXfindEntryField, self.Xpos)
        update_entry(self.VisYfindEntryField, self.Ypos)
        update_entry(self.VisRZfindEntryField, r)
        update_entry(self.VisXpixfindEntryField, x)
        update_entry(self.VisYpixfindEntryField, y)
        update_entry(self.SP_1_E1_EntryField, self.Xpos)
        update_entry(self.SP_1_E2_EntryField, self.Ypos)
        update_entry(self.SP_1_E3_EntryField, r)

    def viscalc(self):
        # Retrieve and convert an entry field's value to float.
        def get_entry_float(entry_field):
            return float(entry_field.get())

        # Calculate the millimeter position based on pixel and millimeter ranges and the target pixel value.
        def calculate_mm_position(origin_pix, end_pix, origin_mm, end_mm, target_pix):
            pixel_range = end_pix - origin_pix
            ratio = (target_pix - origin_pix) / pixel_range
            mm_range = end_mm - origin_mm
            return origin_mm + (mm_range * ratio)

        # Retrieve origin and end positions in both pixel and millimeter units
        VisOrigXpix = get_entry_float(self.VisX1PixEntryField)
        VisOrigXmm = get_entry_float(self.VisX1RobEntryField)
        VisOrigYpix = get_entry_float(self.VisY1PixEntryField)
        VisOrigYmm = get_entry_float(self.VisY1RobEntryField)

        VisEndXpix = get_entry_float(self.VisX2PixEntryField)
        VisEndXmm = get_entry_float(self.VisX2RobEntryField)
        VisEndYpix = get_entry_float(self.VisY2PixEntryField)
        VisEndYmm = get_entry_float(self.VisY2RobEntryField)

        # Target pixel coordinates to be converted
        x = get_entry_float(self.VisRetXpixEntryField)
        y = get_entry_float(self.VisRetYpixEntryField)

        # Calculate mm positions for x and y based on pixel inputs
        xMMpos = calculate_mm_position(VisOrigXpix, VisEndXpix, VisOrigXmm, VisEndXmm, x)
        yMMpos = calculate_mm_position(VisOrigYpix, VisEndYpix, VisOrigYmm, VisEndYmm, y)

        return xMMpos, yMMpos

    ## Define function to show frame ##

    def show_frame(self):
        if self.cam_on:
            ret, frame = self.cap.read()

            if ret:
                # Convert the frame to RGB and resize for display
                cv2image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                img = Image.fromarray(cv2image).resize((480, 320))
                imgtk = ImageTk.PhotoImage(image=img)
                
                self.live_lbl.imgtk = imgtk
                self.live_lbl.configure(image=imgtk)

            # Schedule the next frame update
            self.live_lbl.after(10, self.show_frame)

    def start_vid(self):
        self.stop_vid()  # Ensure any previous video capture is stopped
        self.cam_on = True

        # Get the selected camera index
        selectedCam = self.camList.index(self.visoptions.get()) if self.visoptions.get() in self.camList else 0
        self.cap = cv2.VideoCapture(selectedCam)  # Open the selected camera

        self.show_frame()

    def stop_vid(self):
        self.cam_on = False

        if self.cap and self.cap.isOpened():
            self.cap.release()

    ## Define function to show frame ##

    def take_pic(self):
        # Capture frame from selected camera
        if self.cam_on:
            ret, frame = self.cap.read()
        else:
            selectedCam = self.camList.index(self.visoptions.get())
            self.cap = cv2.VideoCapture(selectedCam)
            ret, frame = self.cap.read()

        # Apply brightness and contrast adjustments
        brightness = int(self.VisBrightSlide.get())
        contrast = int(self.VisContrastSlide.get())
        zoom = int(self.VisZoomSlide.get())

        frame = np.int16(frame) * (contrast / 127 + 1) - contrast + brightness
        frame = np.clip(frame, 0, 255).astype(np.uint8)
        cv2image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Calculate cropping bounds
        height, width = cv2image.shape
        centerX, centerY = height // 2, width // 2
        radiusX, radiusY = int(zoom * height / 100), int(zoom * width / 100)
        cropped = cv2image[centerX - radiusX:centerX + radiusX, centerY - radiusY:centerY + radiusY]
        cv2image = cv2.resize(cropped, (width, height))

        # Update background average color based on autoBG setting
        if self.autoBG.get():
            bg_points = [
                cv2image[int(self.VisX1PixEntryField.get()), int(self.VisY1PixEntryField.get())],
                cv2image[int(self.VisX1PixEntryField.get()), int(self.VisY2PixEntryField.get())],
                cv2image[int(self.VisX2PixEntryField.get()), int(self.VisY2PixEntryField.get())]
            ]
            BGavg = (avg := int(np.mean(bg_points)), avg, avg)
            background = avg
            self.VisBacColorEntryField.configure(state='enabled')
            self.VisBacColorEntryField.delete(0, 'end')
            self.VisBacColorEntryField.insert(0, str(BGavg))
            self.VisBacColorEntryField.configure(state='disabled')
        else:
            temp = self.VisBacColorEntryField.get()
            background = int(temp[temp.find("(") + 1 : temp.find(",")])

        # Apply background mask to image
        mask = np.ones_like(cv2image) * background
        mask[self.mY1:self.mY2, self.mX1:self.mX2] = cv2image[self.mY1:self.mY2, self.mX1:self.mX2]
        cv2image = mask

        # Update UI with processed image
        img = Image.fromarray(cv2image).resize((640, 480))
        imgtk = ImageTk.PhotoImage(image=img)
        self.vid_lbl.imgtk = imgtk
        self.vid_lbl.configure(image=imgtk)

        # Save the image
        cv2.imwrite('curImage.jpg', cv2image)

    def mask_pic(self):
        # Capture frame from selected camera
        if self.cam_on:
            ret, frame = self.cap.read()
        else:
            selectedCam = self.camList.index(self.visoptions.get())
            self.cap = cv2.VideoCapture(selectedCam)
            ret, frame = self.cap.read()

        # Apply brightness and contrast adjustments
        brightness = int(self.VisBrightSlide.get())
        contrast = int(self.VisContrastSlide.get())
        zoom = int(self.VisZoomSlide.get())

        frame = np.int16(frame) * (contrast / 127 + 1) - contrast + brightness
        frame = np.clip(frame, 0, 255).astype(np.uint8)
        cv2image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Crop and resize based on zoom level
        height, width = cv2image.shape
        centerX, centerY = height // 2, width // 2
        radiusX, radiusY = int(zoom * height / 100), int(zoom * width / 100)
        cropped = cv2image[centerX - radiusX:centerX + radiusX, centerY - radiusY:centerY + radiusY]
        cv2image = cv2.resize(cropped, (width, height))

        # Save the image
        cv2.imwrite('curImage.jpg', cv2image)

    def mask_crop(self, event, x, y, flags, param):
        self.cropDone = False

        # Set a background color based on autoBG toggle.
        def handle_bg_color():
            autoBGVal = int(self.autoBG.get())
            if autoBGVal == 1:
                BG1 = self.oriImage[int(self.VisX1PixEntryField.get())][int(self.VisY1PixEntryField.get())]
                BG2 = self.oriImage[int(self.VisX1PixEntryField.get())][int(self.VisY2PixEntryField.get())]
                BG3 = self.oriImage[int(self.VisX2PixEntryField.get())][int(self.VisY2PixEntryField.get())]
                avg = int(mean([BG1, BG2, BG3]))
                BGavg = (avg, avg, avg)
                self.VisBacColorEntryField.configure(state='enabled')
                self.VisBacColorEntryField.delete(0, 'end')
                self.VisBacColorEntryField.insert(0, str(BGavg))
                self.VisBacColorEntryField.configure(state='disabled')
                return avg
            else:
                return eval(self.VisBacColorEntryField.get())

        # Apply the selected background to areas outside the crop region.
        def crop_image_with_bg():
            h, w = self.oriImage.shape[:2]
            for y in range(h):
                for x in range(w):
                    if x >= self.mX2 or x <= self.mX1 or y <= self.mY1 or y >= self.mY2:
                        self.oriImage[y, x] = background

        # Update and display the image on the UI.
        def update_displayed_image():
            img = Image.fromarray(self.oriImage)
            imgtk = ImageTk.PhotoImage(image=img)
            self.vid_lbl.imgtk = imgtk
            self.vid_lbl.configure(image=imgtk)
            filename = 'curImage.jpg'
            cv2.imwrite(filename, self.oriImage)
            cv2.destroyAllWindows()

        # Mouse button down event
        if not self.button_down and event == cv2.EVENT_LBUTTONDOWN:
            self.x_start, self.y_start = x, y
            self.cropping, self.button_down = True, True
            self.box_points[:] = [(x, y)]

        # Mouse is moving
        elif self.button_down and event == cv2.EVENT_MOUSEMOVE and self.cropping:
            image_copy = self.oriImage.copy()
            self.x_end, self.y_end = x, y
            cv2.rectangle(image_copy, self.box_points[0], (x, y), (0, 255, 0), 2)
            cv2.imshow("image", image_copy)

        # Mouse button up event
        elif event == cv2.EVENT_LBUTTONUP:
            self.button_down = False
            self.cropping = False
            self.box_points.append((x, y))
            self.x_end, self.y_end = x, y
            self.mX1, self.mY1, self.mX2, self.mY2 = self.x_start+3, self.y_start+3, self.x_end-3, self.y_end-3

            # Set background, crop image, and update display
            background = handle_bg_color()
            crop_image_with_bg()
            update_displayed_image()

    def selectMask(self):
        def setup_mask_window():
            cv2.namedWindow("image")
            cv2.setMouseCallback("image", self.mask_crop)
            cv2.imshow("image", oriImage)

        self.button_down = False
        self.x_start, self.y_start, self.x_end, self.y_end = 0, 0, 0, 0  # Initialize coordinates
        self.mask_pic()  # Call external function for initial masking setup

        oriImage = cv2.imread('curImage.jpg').copy()  # Load and duplicate the current image
        setup_mask_window()  # Set up the window and callback for cropping

    def mouse_crop(self, event, x, y, flags, param):
        # Draw a rectangle on a copy of the image and display it.
        def update_image_with_rectangle(image, start, end, color=(0, 255, 0), thickness=2):
            image_copy = image.copy()
            cv2.rectangle(image_copy, start, end, color, thickness)
            cv2.imshow("image", image_copy)

        self.cropDone = False

        if not self.button_down and event == cv2.EVENT_LBUTTONDOWN:
            # Start cropping
            self.x_start, self.y_start, self.x_end, self.y_end = x, y, x, y
            self.cropping = True
            self.button_down = True
            box_points = [(x, y)]

        elif self.button_down and event == cv2.EVENT_MOUSEMOVE and self.cropping:
            # Update rectangle as mouse moves
            self.x_end, self.y_end = x, y
            update_image_with_rectangle(self.oriImage, box_points[0], (self.x_end, self.y_end))

        elif event == cv2.EVENT_LBUTTONUP:
            # Finish cropping
            self.button_down = False
            box_points.append((x, y))
            cv2.rectangle(self.oriImage, box_points[0], box_points[1], (0, 255, 0), 2)
            cv2.imshow("image", self.oriImage)

            # Record final crop coordinates and set ROI
            self.x_end, self.y_end = x, y
            self.cropping = False
            refPoint = [(self.x_start + 3, self.y_start + 3), (self.x_end - 3, self.y_end - 3)]

            if len(refPoint) == 2:
                # Crop and save the region of interest
                roi = self.oriImage[refPoint[0][1]:refPoint[1][1], refPoint[0][0]:refPoint[1][0]]
                cv2.imshow("Cropped", roi)

                # Prompt for template name and save the cropped image
                template_name = self.simpledialog.askstring(title="Teach Vision Object", prompt="Save Object As:")
                if template_name:
                    cv2.imwrite(f"{template_name}.jpg", roi)
                cv2.destroyAllWindows()
                self.updateVisOp()

    def selectTemplate(self):
        self.button_down = False
        self.x_start = self.y_start = self.x_end = self.y_end = 0

        image = cv2.imread("curImage.jpg")
        oriImage = image.copy()

        cv2.namedWindow("image")
        cv2.setMouseCallback("image", self.mouse_crop)
        cv2.imshow("image", image)

    def snapFind(self):
        self.take_pic()

        template = self.selectedTemplate.get()
        min_score = float(self.VisScoreEntryField.get()) * 0.01
        autoBGVal = int(self.autoBG.get())
        background = self.BGavg if autoBGVal == 1 else eval(self.VisBacColorEntryField.get())

        if autoBGVal == 1:
            self.VisBacColorEntryField.configure(state="normal")
            self.VisBacColorEntryField.delete(0, "end")
            self.VisBacColorEntryField.insert(0, str(self.BGavg))
            self.VisBacColorEntryField.configure(state="disabled")

        self.visFind(template, min_score, background)

    def rotate_image(img, angle, background):
        image_center = tuple(np.array(img.shape[1::-1]) / 2)
        rot_mat = cv2.getRotationMatrix2D(image_center, -angle, 1.0)
        return cv2.warpAffine(
            img, rot_mat, img.shape[1::-1],
            borderMode=cv2.BORDER_CONSTANT, borderValue=background, flags=cv2.INTER_LINEAR
        )

    def visFind(self, template, min_score, background):
        def set_background():
            if background == "Auto":
                background_val = self.BGavg
                self.VisBacColorEntryField.configure(state='enabled')
                self.VisBacColorEntryField.delete(0, 'end')
                self.VisBacColorEntryField.insert(0, str(self.BGavg))
                self.VisBacColorEntryField.configure(state='disabled')
                return background_val
            return eval(self.VisBacColorEntryField.get())

        def rotate_and_match_template(angle_step, method):
            best_score, best_angle, best_loc, best_dims = 0, 0, (0, 0), (0, 0)
            for angle in range(0, 360, angle_step):
                rotated_template = self.rotate_image(img2, angle, background)
                res = cv2.matchTemplate(img1, rotated_template, method)
                _, max_val, _, max_loc = cv2.minMaxLoc(res)
                if max_val > best_score:
                    best_score, best_angle, best_loc = max_val, angle, max_loc
                    best_dims = rotated_template.shape[1::-1]
            return best_score, best_angle, best_loc, best_dims

        def refine_angle_search(method):
            high_score, high_angle, high_loc, high_dims = 0, 0, (0, 0), (0, 0)
            for angle_offset in [0, 120, 240]:
                score, angle, loc, dims = rotate_and_match_template(120, method)
                if score > high_score:
                    high_score, high_angle, high_loc, high_dims = score, angle, loc, dims

            angle_step = high_angle / 2
            while angle_step >= 0.9:
                for next_angle in [high_angle + angle_step, high_angle - angle_step]:
                    score, angle, loc, dims = rotate_and_match_template(1, method)
                    if score > high_score:
                        high_score, high_angle, high_loc, high_dims = score, angle, loc, dims
                angle_step /= 2
            return high_score, high_angle, high_loc, high_dims

        def normalize_angle(angle):
            angle = angle - 360 if angle > 180 else angle
            if self.pick180.get() == "1":
                angle += -180 if angle > 90 else (180 if angle < -90 else 0)
            limit = self.J6PosLim if angle > 0 else self.J6NegLim
            if self.pickClosest.get() == "0" and abs(angle) > limit:
                return "fail"
            return min(angle, limit)

        def draw_alignment_lines(angle, center):
            green = (0, 255, 0)
            for offset in [-90, 90, 0, 180]:
                end_x = int(center[0] + 30 * math.cos(math.radians(angle + offset)))
                end_y = int(center[1] + 30 * math.sin(math.radians(angle + offset)))
                cv2.line(img_copy, center, (end_x, end_y), green, 3)

        def update_display():
            img_resized = Image.fromarray(cv2.resize(img_copy, (640, 480)))
            self.vid_lbl.imgtk = ImageTk.PhotoImage(image=img_resized)
            self.vid_lbl.configure(image=self.vid_lbl.imgtk)

        def fail_status():
            cv2.rectangle(img_copy, (5, 5), (635, 475), (255, 0, 0), 5)
            update_display()
            for field in [self.VisRetScoreEntryField, self.VisRetAngleEntryField, self.VisRetXpixEntryField, self.VisRetYpixEntryField]:
                field.delete(0, 'end')
                field.insert(0, "NA" if field != self.VisRetScoreEntryField else str(round(score * 100, 2)))
            return "fail"

        def process_match_success(score, angle, loc, dims):
            angle = normalize_angle(angle)
            xPos, yPos = int(loc[1] + dims[1] / 2), int(loc[0] + dims[0] / 2)
            draw_alignment_lines(angle, (xPos, yPos))
            update_display()
            
            fields_data = [
                (self.VisRetScoreEntryField, str(round(score * 100, 2))),
                (self.VisRetAngleEntryField, str(angle)),
                (self.VisRetXpixEntryField, str(xPos)),
                (self.VisRetYpixEntryField, str(yPos)),
                (self.VisRetXrobEntryField, str(round(self.xMMpos, 2))),
                (self.VisRetYrobEntryField, str(round(self.yMMpos, 2))),
            ]
            for field, data in fields_data:
                field.delete(0, 'end')
                field.insert(0, data)
            self.viscalc()
            return "pass"

        background = set_background()
        img1 = cv2.imread('curImage.jpg')
        img2 = cv2.imread(template)
        img_copy = img1.copy()
        method = cv2.TM_CCOEFF_NORMED
        fullRotVal = int(self.fullRot.get())

        if fullRotVal == 0:
            score, angle, loc, dims = refine_angle_search(method)
        else:
            score, angle, loc, dims = rotate_and_match_template(1, method)

        if score < min_score:
            return fail_status()
        else:
            return process_match_success(score, angle, loc, dims)
    
    def updateVisOp(self):
        selectedTemplate = ctk.StringVar()

        folder = os.path.dirname(sys.executable) if getattr(sys, 'frozen', False) else os.path.dirname(os.path.realpath(__file__))
        
        filelist = [fname for fname in os.listdir(folder) if fname.endswith('.jpg')]

        # Create and place the dropdown menu with file options using CustomTkinter OptionMenu
        self.Visoptmenu = ctk.CTkOptionMenu(
            master=self.tab6, 
            variable=selectedTemplate, 
            values=filelist,
            command=self.VisOpUpdate
        )
        self.Visoptmenu.place(x=390, y=52)

    def VisOpUpdate(self):
        file = self.selectedTemplate.get()

        # Load and convert the image to RGB
        img = cv2.cvtColor(cv2.imread(file, cv2.IMREAD_COLOR), cv2.COLOR_BGR2RGB)

        # Calculate the resizing dimensions to match target pixel area
        TARGET_PIXEL_AREA = 22500
        ratio = img.shape[1] / img.shape[0]
        new_height = int(math.sqrt(TARGET_PIXEL_AREA / ratio) + 0.5)
        new_width = int(new_height * ratio + 0.5)

        # Resize image and update display
        resized_img = cv2.resize(img, (new_width, new_height))
        imgtk = ImageTk.PhotoImage(image=Image.fromarray(resized_img))
        self.template_lbl.imgtk = imgtk
        self.template_lbl.configure(image=imgtk)

    def zeroBrCn(self):
        # Set default coordinates and reset sliders
        mX1, mY1 = 0, 0
        mX2, mY2 = 640, 480
        self.VisBrightSlide.set(0)
        self.VisContrastSlide.set(0)

        self.take_pic()

    def VisUpdateBriCon(self):
        self.take_pic()

    def motion(self, event):
        y = event.x
        x = event.y

        if x <= 240 and y <= 320:
            # Update top-left corner coordinates
            self.VisX1PixEntryField.delete(0, 'end')
            self.VisX1PixEntryField.insert(0, x)
            self.VisY1PixEntryField.delete(0, 'end')
            self.VisY1PixEntryField.insert(0, y)
        elif x > 240:
            # Update bottom-right X coordinate
            self.VisX2PixEntryField.delete(0, 'end')
            self.VisX2PixEntryField.insert(0, x)
        elif y > 320:
            # Update bottom-right Y coordinate
            self.VisY2PixEntryField.delete(0, 'end')
            self.VisY2PixEntryField.insert(0, y)

    def checkAutoBG(self):
        autoBGVal = int(self.autoBG.get())
        # Disable or enable VisBacColorEntryField based on autoBG value
        state = 'disabled' if autoBGVal == 1 else 'enabled'
        self.VisBacColorEntryField.configure(state=state)

    # GCODE defs #

    def gcodeFrame(self):
        # Create and place the CTkFrame
        gcodeframe = ctk.CTkFrame(self.tab7)
        gcodeframe.place(x=300, y=10)

        # Set up the CTkScrollbar
        scrollbar = ctk.CTkScrollbar(gcodeframe)
        scrollbar.pack(side=ctk.RIGHT, fill=ctk.Y)

        self.tab7.gcodeView = ctk.CTkTextbox(gcodeframe, width=105, height=46, yscrollcommand=scrollbar.set)
        self.tab7.gcodeView.bind('<<ListboxSelect>>', self.gcodeViewselect)
        self.tab7.gcodeView.pack()
        
        # Configure the scrollbar to scroll the Listbox
        scrollbar.configure(command=self.tab7.gcodeView.yview)

        # Brief delay to allow the interface to update
        time.sleep(0.1)

    def gcodeViewselect(self, e):
        # Get the selected row in the gcodeView Listbox
        gcodeRow = self.tab7.gcodeView.curselection()[0]
        
        # Update the GcodCurRowEntryField with the selected row index
        self.GcodCurRowEntryField.delete(0, 'end')
        self.GcodCurRowEntryField.insert(0, gcodeRow)

    def loadGcodeProg(self):
        # Set file types for the file dialog
        filetypes = (('G-code files', '*.gcode *.nc *.ngc *.cnc *.tap'),
                    ('Text files', '*.txt'))

        # Open file dialog and get the selected file path
        filename = fd.askopenfilename(title='Open files', initialdir='/', filetypes=filetypes)
        if not filename:
            return  # Exit if no file is selected

        # Update GcodeProgEntryField with the selected filename
        self.GcodeProgEntryField.delete(0, 'end')
        self.GcodeProgEntryField.insert(0, filename)

        # Clear the current contents of gcodeView
        self.tab7.gcodeView.delete(0, END)
        
        # Open and read the G-code file
        with open(filename, "rb") as gcodeProg:
            previtem = b""
            for item in gcodeProg:
                # Remove comments from each line, if present
                commentIndex = item.find(b";")
                item = item[:commentIndex].strip() + b" "

                # Insert only unique lines
                if item != previtem:
                    self.tab7.gcodeView.insert(END, item)
                previtem = item

        # Configure scrollbar for gcodeView
        self.gcodescrollbar.configure(command=self.tab7.gcodeView.yview)

    def SetGcodeStartPos(self):
        # List of entry fields and corresponding position variables
        entry_fields = [
            (self.GC_ST_E1_EntryField, self.XcurPos),
            (self.GC_ST_E2_EntryField, self.YcurPos),
            (self.GC_ST_E3_EntryField, self.ZcurPos),
            (self.GC_ST_E4_EntryField, self.RzcurPos),
            (self.GC_ST_E5_EntryField, self.RycurPos),
            (self.GC_ST_E6_EntryField, self.RxcurPos),
            (self.GC_ST_WC_EntryField, self.WC)
        ]

        # Update each entry field with the corresponding position value
        for entry_field, pos_value in entry_fields:
            entry_field.delete(0, 'end')
            entry_field.insert(0, str(pos_value))

    def MoveGcodeStartPos(self):
        # Calculate positions
        positions = {
            "X": float(self.GC_ST_E1_EntryField.get()) + float(self.GC_SToff_E1_EntryField.get()),
            "Y": float(self.GC_ST_E2_EntryField.get()) + float(self.GC_SToff_E2_EntryField.get()),
            "Z": float(self.GC_ST_E3_EntryField.get()) + float(self.GC_SToff_E3_EntryField.get()),
            "Rz": float(self.GC_ST_E4_EntryField.get()) + float(self.GC_SToff_E4_EntryField.get()),
            "Ry": float(self.GC_ST_E5_EntryField.get()) + float(self.GC_SToff_E5_EntryField.get()),
            "Rx": float(self.GC_ST_E6_EntryField.get()) + float(self.GC_SToff_E6_EntryField.get()),
            "J7": self.J7PosCur,
            "J8": self.J8PosCur,
            "J9": self.J9PosCur,
        }

        # Motion parameters
        speed_params = {
            "speedPrefix": "Sm",
            "Speed": "25",
            "ACCspd": "10",
            "DECspd": "10",
            "ACCramp": "100",
            "WC": self.GC_ST_WC_EntryField.get(),
        }

        # Loop mode
        loop_mode = "".join(
            str(stat.get())
            for stat in [self.J1OpenLoopStat, self.J2OpenLoopStat, self.J3OpenLoopStat, self.J4OpenLoopStat, self.J5OpenLoopStat, self.J6OpenLoopStat]
        )

        # Construct command string
        command = (
            f"MJX{positions['X']}Y{positions['Y']}Z{positions['Z']}"
            f"Rz{positions['Rz']}Ry{positions['Ry']}Rx{positions['Rx']}"
            f"J7{positions['J7']}J8{positions['J8']}J9{positions['J9']}"
            f"{speed_params['speedPrefix']}{speed_params['Speed']}"
            f"Ac{speed_params['ACCspd']}Dc{speed_params['DECspd']}"
            f"Rm{speed_params['ACCramp']}W{speed_params['WC']}Lm{loop_mode}\n"
        )

        # Send and handle command
        self.cmdSentEntryField.delete(0, 'end')
        self.cmdSentEntryField.insert(0, command)
        self.ser.write(command.encode())
        self.ser.flushInput()
        time.sleep(0.1)
        response = self.ser.readline().strip().decode('utf-8')

        if response.startswith("E"):
            self.ErrorHandler(response)
        else:
            self.displayPosition(response)

    def GCstepFwd(self):
        # Update GCode status
        self.GCalmStatusLab.configure(text="GCODE READY", style="OK.TLabel")
        self.GCexecuteRow()

        # Get the currently selected row and total rows
        selected_row = self.tab7.gcodeView.curselection()[0]
        total_rows = self.tab7.gcodeView.index('end')

        # Update colors for executed, current, and pending rows
        for row in range(0, selected_row):
            self.tab7.gcodeView.itemconfig(row, {'fg': 'dodger blue'})
        self.tab7.gcodeView.itemconfig(selected_row, {'fg': 'blue2'})
        for row in range(selected_row + 1, total_rows):
            self.tab7.gcodeView.itemconfig(row, {'fg': 'black'})

        # Update selection for the next row
        self.tab7.gcodeView.selection_clear(0, END)
        next_row = selected_row + 1
        self.tab7.gcodeView.select_set(next_row)

        # Update the current row display field
        try:
            self.GcodCurRowEntryField.delete(0, 'end')
            self.GcodCurRowEntryField.insert(0, next_row)
        except Exception:  # Fallback in case of an error
            self.GcodCurRowEntryField.delete(0, 'end')
            self.GcodCurRowEntryField.insert(0, "---")

    def GCdelete(self):
        filename = self.GcodeFilenameField.get()

        # Validate input
        if not filename:
            messagebox.showwarning("Warning", "Please enter a filename")
            return

        # Prepare and send the delete command
        full_filename = f"{filename}.txt"
        command = f"DGFn{full_filename}\n"
        self.cmdSentEntryField.delete(0, 'end')
        self.cmdSentEntryField.insert(0, command)
        self.ser.write(command.encode())
        self.ser.flushInput()
        time.sleep(0.1)

        # Process response
        response = self.ser.readline().strip().decode('utf-8')
        if response.startswith('E'):
            self.ErrorHandler(response)
            return

        # Handle successful or failed deletion
        if response == "P":
            self.GCalmStatusLab.configure(
                text=f"{full_filename} has been deleted", style="OK.TLabel")
            self.GCread("no")
        elif response == "F":
            self.GCalmStatusLab.configure(
                text=f"{full_filename} was not found", style="Alarm.TLabel")

    def GCread(self, status):
        # Prepare and send the command
        command = "RG\n"
        self.cmdSentEntryField.delete(0, 'end')
        self.cmdSentEntryField.insert(0, command)
        self.ser.write(command.encode())
        self.ser.flushInput()
        time.sleep(0.1)

        # Receive and process the response
        response = self.ser.readline().strip().decode('utf-8')
        if response.startswith('E'):
            self.ErrorHandler(response)
            return

        # Update status if files are found
        if status == "yes":
            self.GCalmStatusLab.configure(text="FILES FOUND ON SD CARD:", fg_color="green")  # Updated for CTkLabel

        # Update the G-code program view
        self.GcodeProgEntryField.delete(0, 'end')
        self.tab7.gcodeView.delete(0, ctk.END)
        for value in response.split(","):
            self.tab7.gcodeView.insert(ctk.END, value)
        self.tab7.gcodeView.pack()
        self.gcodescrollbar.configure(command=self.tab7.gcodeView.yview)

    def GCplay(self):
        filename = self.GcodeFilenameField.get().strip()
        
        if not filename:
            messagebox.showwarning("Warning", "Please enter a valid filename.")
            self.GCalmStatusLab.configure(text="No G-code file specified", style="Alarm.TLabel")
            return
        
        # If filename exists, update status and run the file
        self.GCalmStatusLab.configure(text=f"Running G-code File: {filename}", style="OK.TLabel")

    def GCplayProg(self, Filename):
        self.GCalmStatusLab.configure(text="GCODE FILE RUNNING", fg_color="green")

        def GCthreadPlay():
            # Build the command and update UI fields
            Fn = Filename + ".txt"
            command = "PG" + "Fn" + Fn + "\n"
            self.cmdSentEntryField.delete(0, 'end')
            self.cmdSentEntryField.insert(0, command)

            # Send the command
            self.ser.write(command.encode())
            self.ser.flushInput()
            time.sleep(.1)

            # Process the response
            response = str(self.ser.readline().strip(), 'utf-8')
            if response[:1] == 'E':
                self.ErrorHandler(response)
            else:
                self.displayPosition(response)

                # Update status label based on estop state
                if self.estopActive == True:
                    self.GCalmStatusLab.configure(
                        text="Estop Button was Pressed", fg_color="red")
                else:
                    self.GCalmStatusLab.configure(
                        text="GCODE FILE COMPLETE", fg_color="yellow")

        # Start the process in a separate thread
        GCplay = threading.Thread(target=GCthreadPlay)
        GCplay.start()

    def GCconvertProg(self):
        if self.GcodeProgEntryField.get() == "":
            messagebox.showwarning("warning", "Please Load a Gcode Program")
            return
        if self.GcodeFilenameField.get() == "":
            messagebox.showwarning("warning", "Please Enter a Filename")
            return

        # Prepare command and update UI fields
        Filename = self.GcodeFilenameField.get() + ".txt"
        command = "DG" + "Fn" + Filename + "\n"
        self.cmdSentEntryField.delete(0, 'end')
        self.cmdSentEntryField.insert(0, command)
        self.ser.write(command.encode())
        self.ser.flushInput()
        time.sleep(.1)
        response = str(self.ser.readline().strip(), 'utf-8')
        last = self.tab7.gcodeView.index('end')
        for row in range(0, last):
            self.tab7.gcodeView.itemconfig(row, {'fg': 'black'})

        def GCthreadProg():
            self.prevxVal, self.prevyVal, self.prevzVal = 0, 0, 0
            self.GCstopQueue, self.splineActive = "0", "0"

            try:
                GCselRow = self.tab7.gcodeView.curselection()[0]
                if GCselRow == 0:
                    GCselRow = 1
            except:
                GCselRow = 1
                self.tab7.gcodeView.selection_clear(0, END)
                self.tab7.gcodeView.select_set(GCselRow)

            self.tab7.GCrunTrue = 1

            while self.tab7.GCrunTrue == 1:
                if self.tab7.GCrunTrue == 0:
                    self.GCalmStatusLab.configure(
                        text="GCODE CONVERSION STOPPED", style="Alarm.TLabel")
                    break

                self.GCalmStatusLab.configure(
                    text="GCODE CONVERSION RUNNING", style="OK.TLabel")

                self.GCrowinproc = 1
                self.GCexecuteRow()

                while self.GCrowinproc == 1:
                    time.sleep(.1)

                try:
                    GCselRow = self.tab7.gcodeView.curselection()[0]
                    self.tab7.gcodeView.itemconfig(GCselRow, {'fg': 'blue2'})
                    self.tab7.gcodeView.selection_clear(0, END)
                    GCselRow += 1
                    self.tab7.gcodeView.select_set(GCselRow)
                    self.GcodCurRowEntryField.delete(0, 'end')
                    self.GcodCurRowEntryField.insert(0, GCselRow)
                except:
                    self.GcodCurRowEntryField.delete(0, 'end')
                    self.GcodCurRowEntryField.insert(0, "---")
                    self.tab7.GCrunTrue = 0
                    self.GCalmStatusLab.configure(
                        text="GCODE CONVERSION STOPPED", style="Alarm.TLabel")

        GCt = threading.Thread(target=GCthreadProg)
        GCt.start()

    def GCstopProg(self):
        self.tab7.GCrunTrue = 0
        self.GCalmStatusLab.configure(text="GCODE CONVERSION STOPPED", style="Alarm.TLabel")

        if self.splineActive == 1:
            self.splineActive = "0"
            if self.GCstopQueue == "1":
                self.GCstopQueue = "0"
                stop()

            if self.moveInProc == 1:
                self.moveInProc = 2

            command = "SS\n"
            self.cmdSentEntryField.delete(0, 'end')
            self.cmdSentEntryField.insert(0, command)
            self.ser.write(command.encode())
            self.ser.flushInput()
            response = str(self.ser.readline().strip(), 'utf-8')

            if response[:1] == 'E':
                self.ErrorHandler(response)
            else:
                self.displayPosition(response)

    def GCexecuteRow(self):
        def parse_coordinate(command, axis, default_val):
            if axis in command:
                value = command[command.find(axis) + 1:]
                value = value[:value.find(" ")] if " " in value else value
                value = str(round(float(value), 3))
                if self.inchTrue:
                    value = str(float(value) * 25.4)
                value = str(round(float(default_val) + float(value), 3))
            else:
                try:
                    value = str(default_val) if eval(f"prev{axis.lower()}Val") == 0 else eval(f"prev{axis.lower()}Val")
                except:
                    value = str(default_val)
            return value

        def create_gcode_command(xVal, yVal, zVal, rzVal, ryVal, rxVal, J7Val, speed):
            ACCspd, DECspd, ACCramp, Rounding = ".1", ".1", "100", "0"
            WC = self.GC_ST_WC_EntryField.get()
            LoopMode = "111111"
            Filename = self.GcodeFilenameField.get() + ".txt"
            return (
                f"WCX{xVal}Y{yVal}Z{zVal}Rz{rzVal}Ry{ryVal}Rx{rxVal}J7{J7Val}J8{self.J8PosCur}J9{self.J9PosCur}"
                f"Sm{speed}Ac{ACCspd}Dc{DECspd}Rm{ACCramp}Rnd{Rounding}W{WC}Lm{LoopMode}Fn{Filename}\n"
            )

        GCstartTime = time.time()
        GCselRow = self.tab7.gcodeView.curselection()[0]
        self.tab7.gcodeView.see(GCselRow + 2)
        command = self.tab7.gcodeView.get(self.tab7.gcodeView.curselection()[0]).decode()
        cmdType, subCmd = command[:1], command[1:command.find(" ")].rstrip()

        if cmdType == "F":
            self.gcodeSpeed = command[command.find("F") + 1:]

        elif cmdType == "G":
            if subCmd in {"20", "21"}:
                self.inchTrue = subCmd == "20"

            elif subCmd in {"90", "91", "28"}:
                xVal, yVal, zVal = [
                    str(float(eval(f"GC_ST_E{i}_EntryField.get()")) + float(eval(f"GC_SToff_E{i}_EntryField.get()")))
                    for i in range(1, 4)
                ]
                rzVal, ryVal, rxVal = [
                    str(float(eval(f"GC_ST_E{i}_EntryField.get()")) + float(eval(f"GC_SToff_E{i}_EntryField.get()")))
                    for i in range(4, 7)
                ]
                command = create_gcode_command(xVal, yVal, zVal, rzVal, ryVal, rxVal, str(self.J7PosCur), "25")
                self.cmdSentEntryField.delete(0, 'end')
                self.cmdSentEntryField.insert(0, command)
                self.ser.write(command.encode())
                self.ser.flushInput()
                time.sleep(.1)
                response = str(self.ser.readline().strip(), 'utf-8')
                if response.startswith('E'):
                    self.ErrorHandler(response)
                    self.GCstopProg()
                    self.tab7.GCrunTrue = 0
                    self.GCalmStatusLab.configure(text="UNABLE TO WRITE TO SD CARD", style="Alarm.TLabel")
                else:
                    self.displayPosition(response)

            elif subCmd in {"0", "1"}:
                xVal = parse_coordinate(command, "X", self.XcurPos)
                yVal = parse_coordinate(command, "Y", self.YcurPos)
                zVal = parse_coordinate(command, "Z", self.ZcurPos)

                rzVal = parse_coordinate(command, "A", self.RzcurPos)
                ryVal = parse_coordinate(command, "B", self.RycurPos)
                rxVal = parse_coordinate(command, "C", self.RxcurPos)

                J7Val = parse_coordinate(command, "E", self.J7PosCur)

                speed = self.gcodeSpeed if subCmd == "1" else self.speedEntryField.get()
                command = create_gcode_command(xVal, yVal, zVal, rzVal, ryVal, rxVal, J7Val, speed)
                self.prevxVal, self.prevyVal, self.prevzVal = xVal, yVal, zVal
                self.cmdSentEntryField.delete(0, 'end')
                self.cmdSentEntryField.insert(0, command)
                self.ser.write(command.encode())
                self.ser.flushInput()
                time.sleep(.05)
                response = str(self.ser.readline().strip(), 'utf-8')
                if response.startswith('E'):
                    self.ErrorHandler(response)
                    self.tab7.GCrunTrue = 0
                    self.GCalmStatusLab.configure(text="UNABLE TO WRITE TO SD CARD", style="Alarm.TLabel")
                else:
                    self.displayPosition(response)

        self.GCrowinproc = 0

## Run the application ##
if __name__ == "__main__":
    app = RobotArmApp()
    app.root.mainloop()
