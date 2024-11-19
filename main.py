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
        self.root.iconbitmap(r'EE.ico')
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

        self.xbcStatusLab = ctk.CTkLabel(self.tab1, text="Xbox OFF")
        self.xbcStatusLab.place(x=1270, y=80)

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
            self.almStatusLab.config(text="SYSTEM READY", style="OK.TLabel")
            self.almStatusLab2.config(text="SYSTEM READY", style="OK.TLabel")

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
            self.almStatusLab.config(text=error_message, style="Alarm.TLabel")
            self.almStatusLab2.config(text=error_message, style="Alarm.TLabel")

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
            self.almStatusLab.config(text="SYSTEM READY", style="OK.TLabel")
            self.almStatusLab2.config(text="SYSTEM READY", style="OK.TLabel")

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
        self.almStatusLab.config(text=statusText, style="OK.TLabel")
        self.almStatusLab2.config(text=statusText, style="OK.TLabel")

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
                [TFxEntryField, TFyEntryField, TFzEntryField, 
                TFrzEntryField, TFryEntryField, TFrxEntryField],
                [xVal, yVal, zVal, rzVal, ryVal, rxVal]):
            field.delete(0, 'end')
            field.insert(0, value)

        # Format and send the command
        formattedCommand = f"TF A{xVal} B{yVal} C{zVal} D{rzVal} E{ryVal} F{rxVal}\n"
        cmdSentEntryField.delete(0, 'end')
        cmdSentEntryField.insert(0, formattedCommand)
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
            errorStatusLabel.config(text=response, style="Error.TLabel")
            print(f"Error: {response}")
        else:
            # Display position data from the response
            try:
                position_fields = {
                    "X": PositionXField, "Y": PositionYField, "Z": PositionZField,
                    "Rz": PositionRzField, "Ry": PositionRyField, "Rx": PositionRxField
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
                errorStatusLabel.config(text=errorMsg, style="Error.TLabel")
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
            self.errorStatusLabel.config(text=response, style="Error.TLabel")
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
                self.errorStatusLabel.config(text=errorMsg, style="Error.TLabel")
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
            self.errorStatusLabel.config(text=response, style="Error.TLabel")
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
                self.errorStatusLabel.config(text=errorMsg, style="Error.TLabel")
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
            self.errorStatusLabel.config(text=response, style="Error.TLabel")
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
                self.errorStatusLabel.config(text=error_msg, style="Error.TLabel")

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
            self.errorStatusLabel.config(text=response, style="Error.TLabel")  # Display error in UI
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
            self.errorStatusLabel.config(text=errorMsg, style="Error.TLabel")

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
            self.errorStatusLabel.config(text=response, style="Error.TLabel")
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
            self.errorStatusLabel.config(text=errorMsg, style="Error.TLabel")

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
            self.errorStatusLabel.config(text=response, style="Error.TLabel")
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
                self.errorStatusLabel.config(text=f"Display Error: {str(e)}", style="Error.TLabel")

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
            self.errorStatusLabel.config(text=response, style="Error.TLabel")
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
                self.errorStatusLabel.config(text=f"Display Error: {str(e)}", style="Error.TLabel")

    def handleMoveA(self, command):
        # Start move if not already in process
        if self.moveInProc == 0:
            self.moveInProc = 1

        # Check command validity
        if command.startswith("Move A End"):
            self.almStatusLab.config(text="Move A must start with a Mid followed by End", style="Alarm.TLabel")
            self.almStatusLab2.config(text="Move A must start with a Mid followed by End", style="Alarm.TLabel")
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
            self.errorStatusLabel.config(text=response, style="Error.TLabel")
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
                self.errorStatusLabel.config(text=f"Display Error: {str(e)}", style="Error.TLabel")

    def handleMoveC(self, command):
        if self.moveInProc == 0:
            self.moveInProc = 1

        # Check command format
        subCmd = command[:10]
        if subCmd in ["Move C Sta", "Move C Pla"]:
            # Inline showError logic
            message = "Move C must start with a Center followed by Start & Plane"
            self.almStatusLab.config(text=message, style="Alarm.TLabel")
            self.almStatusLab2.config(text=message, style="Alarm.TLabel")
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
            self.errorStatusLabel.config(text=response, style="Error.TLabel")
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
                self.errorStatusLabel.config(text=errorMsg, style="Error.TLabel")

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
            self.almStatusLab.config(text=label_text, style=label_style)
            self.almStatusLab2.config(text=label_text, style=label_style)

        def toggle_xbox():
            if self.xboxUse == 0:
                self.xboxUse = 1
                self.mainMode, self.jogMode, self.grip = 1, 1, 0
                update_status('JOGGING JOINTS 1 & 2')
                self.xbcStatusLab.config(text='Xbox ON')
                self.ChgDis(2)
            else:
                self.xboxUse = 0
                update_status('XBOX CONTROLLER OFF')
                self.xbcStatusLab.config(text='Xbox OFF')

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
            self.almStatusLab.config(text="SYSTEM READY", style="OK.TLabel")
            self.almStatusLab2.config(text="SYSTEM READY", style="OK.TLabel")

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
            self.almStatusLab.config(text="SYSTEM READY", style="OK.TLabel")
            self.almStatusLab2.config(text="SYSTEM READY", style="OK.TLabel")

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
            self.almStatusLab.config(text="SYSTEM READY", style="OK.TLabel")
            self.almStatusLab2.config(text="SYSTEM READY", style="OK.TLabel")
        
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
            self.almStatusLab.config(text="SYSTEM READY", style="OK.TLabel")
            self.almStatusLab2.config(text="SYSTEM READY", style="OK.TLabel")
        
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
            self.almStatusLab.config(text="Please enter an input, register number or COM Port", style="Alarm.TLabel")
            
        inputVal = self.IfInputEntryField.get()
        destVal = self.IfDestEntryField.get()
        prefix = ""

        if option == "Input":
            if inputVal in ["0", "1"]:
                prefix = f"If Input # {variable} = {inputVal} :"
            else:
                localErrorFlag = True
                self.almStatusLab.config(text="Please enter a 1 or 0 for the = value", style="Alarm.TLabel")
        
        elif option == "Register":
            if not inputVal:
                localErrorFlag = True
                self.almStatusLab.config(text="Please enter a register number", style="Alarm.TLabel")
            prefix = f"If Register # {variable} = {inputVal} :"

        elif option == "COM Device":
            if not inputVal:
                localErrorFlag = True
                self.almStatusLab.config(text="Please enter expected COM device input", style="Alarm.TLabel")
            prefix = f"If COM Device # {variable} = {inputVal} :"
        
        if selection == "Call Prog":
            if not destVal:
                localErrorFlag = True
                self.almStatusLab.config(text="Please enter a program name", style="Alarm.TLabel")
            value = f"{prefix} Call Prog {destVal}"
        
        elif selection == "Jump Tab":
            if not destVal:
                localErrorFlag = True
                self.almStatusLab.config(text="Please enter a destination tab", style="Alarm.TLabel")
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
            self.scrollbar.config(command=self.tab1.progView.yview)
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
            self.almStatusLab.config(text=message, style=style)
            self.almStatusLab2.config(text=message, style=style)
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
            self.almStatusLab.config(text=message, style="OK.TLabel")
            self.almStatusLab2.config(text=message, style="OK.TLabel")
        else:
            message = f"J{joint_id} Calibration Failed"
            self.almStatusLab.config(text=message, style="Alarm.TLabel")
            self.almStatusLab2.config(text=message, style="Alarm.TLabel")
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
        self.almStatusLab.config(text=status_text, style="Warn.TLabel")
        self.almStatusLab2.config(text=status_text, style="Warn.TLabel")
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
        self.almStatusLab.config(text=status_message, style="Warn.TLabel")
        self.almStatusLab2.config(text=status_message, style="Warn.TLabel")

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
        self.almStatusLab.config(text=status_message, style="Warn.TLabel")
        self.almStatusLab2.config(text=status_message, style="Warn.TLabel")

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
            self.almStatusLab.config(text=message, style="Warn.TLabel")
            self.almStatusLab2.config(text=message, style="Warn.TLabel")

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
            self.almStatusLab.config(text=message, style="Alarm.TLabel")
            self.almStatusLab2.config(text=message, style="Alarm.TLabel")
            self.GCalmStatusLab.config(text=message, style="Alarm.TLabel")

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
            label.config(text=text, style=style)

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
            label.config(text=text, style=style)

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
            label.config(text=text, style=style)

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
                
                # Update customtkinter label with the new frame
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

        # Configure the CTkListbox (if `customtkinter` lacks a CTkListbox, you may have to fall back to Listbox)
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
        self.gcodescrollbar.config(command=self.tab7.gcodeView.yview)

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
        self.GCalmStatusLab.config(text="GCODE READY", style="OK.TLabel")
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
            self.GCalmStatusLab.config(
                text=f"{full_filename} has been deleted", style="OK.TLabel")
            self.GCread("no")
        elif response == "F":
            self.GCalmStatusLab.config(
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
            self.GCalmStatusLab.config(text="No G-code file specified", style="Alarm.TLabel")
            return
        
        # If filename exists, update status and run the file
        self.GCalmStatusLab.config(text=f"Running G-code File: {filename}", style="OK.TLabel")

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
                    self.GCalmStatusLab.config(
                        text="GCODE CONVERSION STOPPED", style="Alarm.TLabel")
                    break

                self.GCalmStatusLab.config(
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
                    self.GCalmStatusLab.config(
                        text="GCODE CONVERSION STOPPED", style="Alarm.TLabel")

        GCt = threading.Thread(target=GCthreadProg)
        GCt.start()

    def GCstopProg(self):
        self.tab7.GCrunTrue = 0
        self.GCalmStatusLab.config(text="GCODE CONVERSION STOPPED", style="Alarm.TLabel")

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
                    self.GCalmStatusLab.config(text="UNABLE TO WRITE TO SD CARD", style="Alarm.TLabel")
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
                    self.GCalmStatusLab.config(text="UNABLE TO WRITE TO SD CARD", style="Alarm.TLabel")
                else:
                    self.displayPosition(response)

        self.GCrowinproc = 0

## Run the application ##
if __name__ == "__main__":
    app = RobotArmApp()
    app.root.mainloop()
