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
        # Initialize main window
        self.root = ctk.CTk()
        self.root.title("Robot Arm Software Ver 5.0")
        self.root.geometry('1536x792+0+0')
        self.root.resizable(width=False, height=False)

        # Define paths and key variables
        self.curTheme = "dark"
        self.cropping = False
        self.cam_on = False
        self.cap = None
        self.moveInProc = 0
        self.estopActive = False
        self.posOutreach = False
        self.SplineTrue = False
        self.gcodeSpeed = "10"
        self.inchTrue = False

        # Serial communication variables
        self.ser = None
        self.ser2 = None

        # ExecuteRow variables
        self.J1AngCur = 0
        self.J2AngCur = 0
        self.J3AngCur = 0
        self.J4AngCur = 0
        self.J5AngCur = 0
        self.J6AngCur = 0
        self.calStat = 0
        self.LineDist = 0
        self.Xv = 0
        self.Yv = 0
        self.Zv = 0
        self.commandCalc = False
        self.lastRow = None
        self.lastProg = None

        # Axis limits
        self.axis_limits = {
            "J1": {"pos": 170, "neg": 170},
            "J2": {"pos": 90, "neg": 42},
            "J3": {"pos": 52, "neg": 89},
            "J4": {"pos": 165, "neg": 165},
            "J5": {"pos": 105, "neg": 105},
            "J6": {"pos": 155, "neg": 155},
            "J7": {"pos": 500, "neg": 0},
            "J8": {"pos": 500, "neg": 0},
            "J9": {"pos": 500, "neg": 0}
        }

        # RunProg variables
        self.stopQueue = "0"
        self.splineActive = "0"
        self.rowInProc = False
        self.runTrue = False

        # Variables added for backward compatibility
        self.JogStepsStat = ctk.IntVar()
        self.J1OpenLoopStat = ctk.IntVar()
        self.J2OpenLoopStat = ctk.IntVar()
        self.J3OpenLoopStat = ctk.IntVar()
        self.J4OpenLoopStat = ctk.IntVar()
        self.J5OpenLoopStat = ctk.IntVar()
        self.J6OpenLoopStat = ctk.IntVar()
        self.DisableWristRot = ctk.IntVar()
        self.xboxUse = 0  # or ctk.BooleanVar() if preferred
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

        # Initialize tabs and widgets
        self.init_tabs()
        self.init_widgets()

        # Set protocol for closing
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

    def init_tabs(self):
        # Create Tabview
        self.nb = ctk.CTkTabview(self.root, width=1536, height=792)
        self.nb.pack(fill="both", expand=True)

        # Define and add tabs to tabview
        tab_labels = [
            "Main Controls", "Config Settings", "Kinematics",
            "Inputs Outputs", "Registers", "Vision", "G-Code", "Log"
        ]
        self.tabs = {}
        for label in tab_labels:
            tab = self.nb.add(label)
            self.tabs[label] = tab

    def init_widgets(self):
        # Status labels on the log tab (tab8)
        self.almStatusLab = ctk.CTkLabel(self.tabs["Log"], text="SYSTEM INIT")
        self.almStatusLab.pack()

        # Error log view
        self.ElogView = ctk.CTkTextbox(
            self.tabs["Log"], height=200, wrap="word")
        self.ElogView.pack(fill="both", expand=True)

    def log_message(self, message):
        # Log message with timestamp in ErrorLog view and save to file
        timestamp = datetime.datetime.now().strftime("%B %d %Y - %I:%M%p")
        log_entry = f"{timestamp} - {message}"
        self.ElogView.insert(END, log_entry + "\n")
        self.ElogView.yview(END)

        # Save log entry to a file
        with open("ErrorLog", "ab") as log_file:
            pickle.dump(log_entry, log_file)

    # Startup defs #

    def startup(self):
        self.moveInProc = 0
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
            self.almStatusLab.config(text="SYSTEM READY")
            self.log_message(
                "COMMUNICATIONS STARTED WITH TEENSY 4.1 CONTROLLER")
            time.sleep(0.1)
            self.ser.flushInput()
            self.startup()
        except Exception as e:
            self.almStatusLab.config(text="UNABLE TO ESTABLISH COMMUNICATIONS")
            self.log_message(
                "UNABLE TO ESTABLISH COMMUNICATIONS WITH TEENSY 4.1 CONTROLLER")

    def setCom2(self):
        try:
            port = "COM" + self.com2PortEntryField.get()
            baud = 115200
            self.ser2 = serial.Serial(port, baud)
            self.almStatusLab.config(text="SYSTEM READY")
            self.log_message("COMMUNICATIONS STARTED WITH ARDUINO IO BOARD")
        except Exception as e:
            self.almStatusLab.config(text="UNABLE TO ESTABLISH COMMUNICATIONS")
            self.log_message(
                "UNABLE TO ESTABLISH COMMUNICATIONS WITH ARDUINO IO BOARD")

    def switch_to_dark_theme(self):
        # Apply dark theme with CustomTkinter
        self.curTheme = "dark"
        ctk.set_appearance_mode("dark")
        self.update_styles()

    def switch_to_light_theme(self):
        # Apply light theme with CustomTkinter
        self.curTheme = "light"
        ctk.set_appearance_mode("light")
        self.update_styles()

    def update_styles(self):
        # Set the label color according to the theme
        if self.curTheme == "dark":
            self.almStatusLab.configure(fg_color="light green")
        else:
            self.almStatusLab.configure(fg_color="green")

    # Execution defs #

    def runProg(self):
        def threadProg():
            self.estopActive = False
            self.posOutreach = False
            self.stopQueue = "0"
            self.splineActive = "0"
            try:
                curRow = self.tabs["Main Controls"].progView.curselection()[0]
                if curRow == 0:
                    curRow = 1
            except:
                curRow = 1
                self.tabs["Main Controls"].progView.selection_clear(0, END)
                self.tabs["Main Controls"].progView.select_set(curRow)

            self.runTrue = True
            while self.runTrue:
                if not self.runTrue:
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

                self.rowInProc = True
                self.executeRow()
                while self.rowInProc:
                    time.sleep(0.1)

                selRow = self.tabs["Main Controls"].progView.curselection()[0]
                last = self.tabs["Main Controls"].progView.index('end')
                self.tabs["Main Controls"].progView.selection_clear(0, END)
                selRow += 1
                self.tabs["Main Controls"].progView.select_set(selRow)
                curRow += 1
                time.sleep(0.1)

                try:
                    selRow = self.tabs["Main Controls"].progView.curselection()[
                        0]
                    self.curRowEntryField.delete(0, 'end')
                    self.curRowEntryField.insert(0, selRow)
                except:
                    self.curRowEntryField.delete(0, 'end')
                    self.curRowEntryField.insert(0, "---")
                    self.runTrue = False
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
        selRow = self.tabs["Main Controls"].progView.curselection()[0]
        last = self.tabs["Main Controls"].progView.index('end')
        for row in range(0, selRow):
            self.tabs["Main Controls"].progView.itemconfig(
                row, {'fg': 'dodger blue'})
        self.tabs["Main Controls"].progView.itemconfig(selRow, {'fg': 'blue2'})
        for row in range(selRow + 1, last):
            self.tabs["Main Controls"].progView.itemconfig(
                row, {'fg': 'black'})
        self.tabs["Main Controls"].progView.selection_clear(0, END)
        selRow += 1
        self.tabs["Main Controls"].progView.select_set(selRow)
        try:
            selRow = self.tabs["Main Controls"].progView.curselection()[0]
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
        selRow = self.tabs["Main Controls"].progView.curselection()[0]
        last = self.tabs["Main Controls"].progView.index('end')
        for row in range(0, selRow):
            self.tabs["Main Controls"].progView.itemconfig(
                row, {'fg': 'black'})
        self.tabs["Main Controls"].progView.itemconfig(selRow, {'fg': 'red'})
        for row in range(selRow + 1, last):
            self.tabs["Main Controls"].progView.itemconfig(
                row, {'fg': 'tomato2'})
        self.tabs["Main Controls"].progView.selection_clear(0, END)
        selRow -= 1
        self.tabs["Main Controls"].progView.select_set(selRow)
        try:
            selRow = self.tabs["Main Controls"].progView.curselection()[0]
            self.curRowEntryField.delete(0, 'end')
            self.curRowEntryField.insert(0, selRow)
        except:
            self.curRowEntryField.delete(0, 'end')
            self.curRowEntryField.insert(0, "---")

    def stopProg(self):
        self.runTrue = False
        if self.estopActive:
            self.almStatusLab.configure(
                text="Estop Button was Pressed", fg_color="red")
        elif self.posOutreach:
            self.almStatusLab.configure(
                text="Position Out of Reach", fg_color="red")
        else:
            self.almStatusLab.configure(text="PROGRAM STOPPED", fg_color="red")

    def executeRow(self):
        selRow = self.tabs["Main Controls"].progView.curselection()[0]
        self.tabs["Main Controls"].progView.see(selRow + 2)
        command = self.tabs["Main Controls"].progView.get(selRow).strip()
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
        self.lastRow = self.tabs["Main Controls"].progView.curselection()[0]
        self.lastProg = self.ProgEntryField.get()

        # Extract the program number
        programIndex = command.find("Program -")
        progNum = command[programIndex + 10:].strip()

        self.ProgEntryField.delete(0, 'end')
        self.ProgEntryField.insert(0, progNum)
        self.callProg(progNum)

        time.sleep(0.4)
        # Reset the selection to the start
        self.tabs["Main Controls"].progView.selection_clear(0, END)
        self.tabs["Main Controls"].progView.select_set(0)

    def runGcodeProgram(self, command):
        if self.moveInProc:
            self.moveInProc = 2
        self.lastRow = self.tabs["Main Controls"].progView.curselection()[0]
        self.lastProg = self.ProgEntryField.get()

        # Extract the filename
        programIndex = command.find("Program -")
        filename = command[programIndex + 10:].strip()

        self.manEntryField.delete(0, 'end')
        self.manEntryField.insert(0, filename)
        # Assuming GCplayProg is defined elsewhere in the class
        self.GCplayProg(filename)

        time.sleep(0.4)
        # Reset the selection to the start
        self.tabs["Main Controls"].progView.selection_clear(0, END)
        self.tabs["Main Controls"].progView.select_set(0)

    def returnProgram(self):
        if self.moveInProc:
            self.moveInProc = 2
        lastRow = self.lastRow
        lastProg = self.lastProg

        self.ProgEntryField.delete(0, 'end')
        self.ProgEntryField.insert(0, lastProg)
        self.callProg(lastProg)

        time.sleep(0.4)
        # Re-select the last row
        self.tabs["Main Controls"].progView.selection_clear(0, END)
        self.tabs["Main Controls"].progView.select_set(lastRow)

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
                self.tabs["Main Controls"].lastRow = self.tabs["Main Controls"].progView.curselection()[0]
                self.tabs["Main Controls"].lastProg = self.ProgEntryField.get()
                prog_name = command[command.find("Prog") + 5:] + ".ar"
                callProg(prog_name)

                # Reset selection in progView
                index = 0
                self.tabs["Main Controls"].progView.selection_clear(0, "end")
                self.tabs["Main Controls"].progView.select_set(index)

            elif action == "Jump":
                # Handle "Jump" action directly
                tab_num = command[command.find("Tab") + 4:]
                encoded_tab = ("Tab Number " + tab_num + "\r\n").encode('utf-8')
                index = self.tabs["Main Controls"].progView.get(0, "end").index(encoded_tab) - 1
                self.tabs["Main Controls"].progView.selection_clear(0, "end")
                self.tabs["Main Controls"].progView.select_set(index)

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
            self.tabs["Error Logs"].ElogView.insert(END, error_message)
            error_log = self.tabs["Error Logs"].ElogView.get(0, END)
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
                self.tabs["Main Controls"].lastRow = self.tabs["Main Controls"].progView.curselection()[0]
                self.tabs["Main Controls"].lastProg = self.ProgEntryField.get()
                prog_name = command[command.find("Prog") + 5:] + ".ar"
                callProg(prog_name)
                
                # Reset selection in progView
                index = 0
                self.tabs["Main Controls"].progView.selection_clear(0, "end")
                self.tabs["Main Controls"].progView.select_set(index)

            elif action == "Jump":
                # Handle "Jump" action within this method
                tab_num = command[command.find("Tab") + 4:]
                encoded_tab = ("Tab Number " + tab_num + "\r\n").encode('utf-8')
                index = self.tabs["Main Controls"].progView.get(0, "end").index(encoded_tab) - 1
                self.tabs["Main Controls"].progView.selection_clear(0, "end")
                self.tabs["Main Controls"].progView.select_set(index)

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
                self.tabs["Main Controls"].lastRow = self.tabs["Main Controls"].progView.curselection()[0]
                self.tabs["Main Controls"].lastProg = self.ProgEntryField.get()
                prog_name = command[command.find("Prog") + 5:] + ".ar"
                callProg(prog_name)
                
                # Reset selection in progView
                index = 0
                self.tabs["Main Controls"].progView.selection_clear(0, END)
                self.tabs["Main Controls"].progView.select_set(index)
                
            elif action == "Jump":
                # Handle "Jump" action
                tab_num = command[command.find("Tab") + 4:]
                encoded_tab = ("Tab Number " + tab_num + "\r\n").encode('utf-8')
                
                # Find and select the tab by index in progView
                index = self.tabs["Main Controls"].progView.get(0, "end").index(encoded_tab) - 1
                self.tabs["Main Controls"].progView.selection_clear(0, END)
                self.tabs["Main Controls"].progView.select_set(index)

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
            index = self.tabs["Main Controls"].progView.get(0, "end").index(encoded_tab) - 1
            self.tabs["Main Controls"].progView.selection_clear(0, END)
            self.tabs["Main Controls"].progView.select_set(index)

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
            index = self.tabs["Main Controls"].progView.get(0, "end").index(encoded_tab) - 1
            self.tabs["Main Controls"].progView.selection_clear(0, END)
            self.tabs["Main Controls"].progView.select_set(index)

    def processJumpToRow(self, command):
        if self.moveInProc:
            self.moveInProc = 2

        # Extract tab number directly
        start_str = "Tab-"
        start_idx = command.find(start_str) + len(start_str)
        tab_num = command[start_idx:]

        # Locate and select the tab in progView
        encoded_tab = ("Tab Number " + tab_num + "\r\n").encode('utf-8')
        index = self.tabs["Main Controls"].progView.get(0, "end").index(encoded_tab)
        self.tabs["Main Controls"].progView.selection_clear(0, END)
        self.tabs["Main Controls"].progView.select_set(index)

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
        ser.write(io_command.encode())
        ser.flushInput()
        time.sleep(0.1)
        ser.read()

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
        ser.write(io_command.encode())
        ser.flushInput()
        time.sleep(0.1)
        ser.read()

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
        ser.write(wait_command.encode())
        ser.flushInput()
        time.sleep(0.1)
        ser.read()

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
        ser.write(wait_command.encode())
        ser.flushInput()
        time.sleep(0.1)
        ser.read()

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
        calRobotAll()
        if calStat == 0:
            stopProg()

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
                [self.TFxEntryField, self.TFyEntryField, self.TFzEntryField, 
                self.TFrzEntryField, self.TFryEntryField, self.TFrxEntryField],
                [xVal, yVal, zVal, rzVal, ryVal, rxVal]):
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
            self.errorStatusLabel.config(text=response, style="Error.TLabel")
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
                self.errorStatusLabel.config(text=errorMsg, style="Error.TLabel")
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
            field = getattr(self, fieldName, None)
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
            field = getattr(self, field_name, None)
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
        LoopMode = ''.join(str(getattr(self, f'J{i}OpenLoopStat').get()) for i in range(1, 7))

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
                    "X": self.PositionXField,
                    "Y": self.PositionYField,
                    "Z": self.PositionZField,
                    "Rz": self.PositionRzField,
                    "Ry": self.PositionRyField,
                    "Rx": self.PositionRxField,
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
            str(getattr(self, f'J{i}OpenLoopStat').get()) for i in range(1, 7))

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
        LoopMode = ''.join(str(getattr(self, f'J{i}OpenLoopStat').get()) for i in range(1, 7))

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
        LoopMode = ''.join(str(getattr(self, f'J{i}OpenLoopStat.get()')) for i in range(1, 7))
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
        # self.manEntryField.delete(0, 'end')
        # self.manEntryField.insert(0, end - start)

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
        LoopMode = ''.join(str(getattr(self, f'J{i}OpenLoopStat.get()')) for i in range(1, 7))

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
        # self.manEntryField.delete(0, 'end')
        # self.manEntryField.insert(0, end - start)

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
        LoopMode = ''.join(str(getattr(self, f'J{i}OpenLoopStat.get()')) for i in range(1, 7))

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
        # self.manEntryField.delete(0, 'end')
        # self.manEntryField.insert(0, end - start)

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
            self.stop()

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
            try:
                # Extract values from response
                xVal = self.extractValue(response, "X")
                yVal = self.extractValue(response, "Y")
                zVal = self.extractValue(response, "Z")
                rzVal = self.extractValue(response, "Rz")
                ryVal = self.extractValue(response, "Ry")
                rxVal = self.extractValue(response, "Rx")

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

    def xbox():
        def update_status(label_text, label_style="Warn.TLabel"):
            almStatusLab.config(text=label_text, style=label_style)
            almStatusLab2.config(text=label_text, style=label_style)

        def toggle_xbox():
            global xboxUse
            if xboxUse == 0:
                xboxUse = 1
                mainMode, jogMode, grip = 1, 1, 0
                update_status('JOGGING JOINTS 1 & 2')
                xbcStatusLab.config(text='Xbox ON')
                ChgDis(2)
            else:
                xboxUse = 0
                update_status('XBOX CONTROLLER OFF')
                xbcStatusLab.config(text='Xbox OFF')

        def handle_event(event):
            global mainMode, jogMode, grip
            increment = float(incrementEntryField.get())

            if event.code == 'ABS_RZ' and event.state >= 100:
                ChgDis(0)
            elif event.code == 'ABS_Z' and event.state >= 100:
                ChgDis(1)
            elif event.code == 'BTN_TR' and event.state == 1:
                ChgSpd(0)
            elif event.code == 'BTN_TL' and event.state == 1:
                ChgSpd(1)
            elif event.code == 'BTN_WEST' and event.state == 1:
                jogMode = handle_joint_mode(mainMode, jogMode)
            elif mainMode == 1:
                handle_joint_jog(event, jogMode, increment)
            elif event.code == 'BTN_SOUTH' and event.state == 1:
                jogMode = handle_cartesian_dir_mode(mainMode, jogMode)
            elif mainMode == 2:
                handle_cartesian_dir_jog(event, jogMode, increment)
            elif event.code == 'BTN_EAST' and event.state == 1:
                jogMode = handle_cartesian_orientation_mode(mainMode, jogMode)
            elif mainMode == 3:
                handle_cartesian_orientation_jog(event, jogMode, increment)
            elif event.code == 'BTN_START' and event.state == 1:
                mainMode = 4
                update_status('JOGGING TRACK')
            elif mainMode == 4:
                handle_track_jog(event, increment)
            elif event.code == 'BTN_NORTH' and event.state == 1:
                teachInsertBelSelected()
            elif event.code == 'BTN_SELECT' and event.state == 1:
                handle_gripper(grip)
                grip = 1 - grip

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
                (1, 'ABS_HAT0X', 1): J1jogNeg,
                (1, 'ABS_HAT0X', -1): J1jogPos,
                (1, 'ABS_HAT0Y', -1): J2jogNeg,
                (1, 'ABS_HAT0Y', 1): J2jogPos,
                (2, 'ABS_HAT0Y', -1): J3jogNeg,
                (2, 'ABS_HAT0Y', 1): J3jogPos,
                (2, 'ABS_HAT0X', 1): J4jogNeg,
                (2, 'ABS_HAT0X', -1): J4jogPos,
                (3, 'ABS_HAT0Y', -1): J5jogNeg,
                (3, 'ABS_HAT0Y', 1): J5jogPos,
                (3, 'ABS_HAT0X', 1): J6jogNeg,
                (3, 'ABS_HAT0X', -1): J6jogPos
            }
            action = joint_mapping.get((jog_mode, event.code, event.state))
            if action:
                action(increment)

        def handle_cartesian_dir_jog(event, jog_mode, increment):
            cartesian_mapping = {
                (1, 'ABS_HAT0Y', -1): XjogNeg,
                (1, 'ABS_HAT0Y', 1): XjogPos,
                (1, 'ABS_HAT0X', 1): YjogNeg,
                (1, 'ABS_HAT0X', -1): YjogPos,
                (2, 'ABS_HAT0Y', 1): ZjogNeg,
                (2, 'ABS_HAT0Y', -1): ZjogPos
            }
            action = cartesian_mapping.get((jog_mode, event.code, event.state))
            if action:
                action(increment)

        def handle_cartesian_orientation_jog(event, jog_mode, increment):
            orientation_mapping = {
                (1, 'ABS_HAT0X', -1): RxjogNeg,
                (1, 'ABS_HAT0X', 1): RxjogPos,
                (1, 'ABS_HAT0Y', 1): RyjogNeg,
                (1, 'ABS_HAT0Y', -1): RyjogPos,
                (2, 'ABS_HAT0X', 1): RzjogNeg,
                (2, 'ABS_HAT0X', -1): RzjogPos
            }
            action = orientation_mapping.get((jog_mode, event.code, event.state))
            if action:
                action(increment)

        def handle_track_jog(event, increment):
            if event.code == 'ABS_HAT0X' and event.state == 1:
                J7jogPos(increment)
            elif event.code == 'ABS_HAT0X' and event.state == -1:
                J7jogNeg(increment)

        def handle_gripper(grip_state):
            outputNum = DO1offEntryField.get() if grip_state == 0 else DO1onEntryField.get()
            command = ("OFX" if grip_state == 0 else "ONX") + outputNum + "\n"
            ser2.write(command.encode())
            ser2.flushInput()
            time.sleep(0.1)
            ser2.read()

        def threadxbox():
            toggle_xbox()
            while xboxUse == 1:
                try:
                    events = get_gamepad()
                    for event in events:
                        handle_event(event)
                except:
                    update_status('XBOX CONTROLLER NOT RESPONDING', "Alarm.TLabel")

        threading.Thread(target=threadxbox).start()

    def ChgDis(val):
        def increase_speed(cur_speed, increment):
            return min(cur_speed + increment, 100)

        def decrease_speed(cur_speed, decrement):
            return max(cur_speed - decrement, 1)

        cur_spd = int(incrementEntryField.get())
        
        if val == 0:  # Increase speed
            cur_spd = increase_speed(cur_spd, 1 if cur_spd < 5 else 5)
        elif val == 1:  # Decrease speed
            cur_spd = decrease_speed(cur_spd, 1 if cur_spd <= 5 else 5)
        elif val == 2:  # Set speed to minimum threshold
            cur_spd = 5

        incrementEntryField.delete(0, 'end')
        incrementEntryField.insert(0, str(cur_spd))

        time.sleep(0.3)

    def ChgSpd(val):
        def increase_speed(cur_speed, increment):
            return min(cur_speed + increment, 100)

        def decrease_speed(cur_speed, decrement):
            return max(cur_speed - decrement, 1)

        cur_spd = int(speedEntryField.get())
        
        if val == 0:  # Increase speed
            cur_spd = increase_speed(cur_spd, 1 if cur_spd < 5 else 5)
        elif val == 1:  # Decrease speed
            cur_spd = decrease_speed(cur_spd, 1 if cur_spd <= 5 else 5)
        elif val == 2:  # Set speed to minimum threshold
            cur_spd = 5

        speedEntryField.delete(0, 'end')
        speedEntryField.insert(0, str(cur_spd))

    def jog_joint(joint_index, value, direction):

        # Helper function to jog a joint in a specific direction.

        """
        Args:
            joint_index (int): The joint index (1-9) to be jogged.
            value (float): The amount to jog the joint.
            direction (str): The direction to jog ("pos" or "neg").
        """
        global xboxUse, J1AngCur, J2AngCur, J3AngCur, J4AngCur, J5AngCur, J6AngCur
        global J7PosCur, J8PosCur, J9PosCur

        # Adjust the appropriate joint angle or position based on the index and direction
        joint_values = [J1AngCur, J2AngCur, J3AngCur, J4AngCur, J5AngCur, J6AngCur, J7PosCur, J8PosCur, J9PosCur]
        if direction == "neg":
            joint_values[joint_index - 1] = str(float(joint_values[joint_index - 1]) - value)
        elif direction == "pos":
            joint_values[joint_index - 1] = str(float(joint_values[joint_index - 1]) + value)
        
        # Update the "SYSTEM READY" status
        checkSpeedVals()
        if xboxUse != 1:
            almStatusLab.config(text="SYSTEM READY", style="OK.TLabel")
            almStatusLab2.config(text="SYSTEM READY", style="OK.TLabel")

        # Determine speed type and prefix
        speedtype = speedOption.get()
        speedPrefix = "Sp" if speedtype in ["mm per Sec", "Percent"] else "Ss"
        if speedtype == "mm per Sec":
            speedEntryField.delete(0, 'end')
            speedEntryField.insert(0, "50")

        # Get values from entry fields
        Speed = speedEntryField.get()
        ACCspd = ACCspeedField.get()
        DECspd = DECspeedField.get()
        ACCramp = ACCrampField.get()
        LoopMode = "".join(str(stat.get()) for stat in [J1OpenLoopStat, J2OpenLoopStat, J3OpenLoopStat, J4OpenLoopStat, J5OpenLoopStat, J6OpenLoopStat])

        # Construct the command string
        command = f"RJ" + "".join(f"{axis}{angle}" for axis, angle in zip("ABCDEF", joint_values[:6])) + \
                f"J7{joint_values[6]}J8{joint_values[7]}J9{joint_values[8]}{speedPrefix}{Speed}Ac{ACCspd}Dc{DECspd}Rm{ACCramp}W{WC}Lm{LoopMode}\n"

        # Update the command entry field and send the command
        cmdSentEntryField.delete(0, 'end')
        cmdSentEntryField.insert(0, command)
        ser.write(command.encode())
        ser.flushInput()
        time.sleep(.1)
        
        # Read and process response
        response = str(ser.readline().strip(), 'utf-8')
        if response.startswith('E'):
            ErrorHandler(response)
        else:
            displayPosition(response)

    # Define specific jog functions for each joint and direction by calling the helper function
    def J1jogNeg(value): jog_joint(1, value, "neg")
    def J1jogPos(value): jog_joint(1, value, "pos")
    def J2jogNeg(value): jog_joint(2, value, "neg")
    def J2jogPos(value): jog_joint(2, value, "pos")
    def J3jogNeg(value): jog_joint(3, value, "neg")
    def J3jogPos(value): jog_joint(3, value, "pos")
    def J4jogNeg(value): jog_joint(4, value, "neg")
    def J4jogPos(value): jog_joint(4, value, "pos")
    def J5jogNeg(value): jog_joint(5, value, "neg")
    def J5jogPos(value): jog_joint(5, value, "pos")
    def J6jogNeg(value): jog_joint(6, value, "neg")
    def J6jogPos(value): jog_joint(6, value, "pos")
    def J7jogNeg(value): jog_joint(7, value, "neg")
    def J7jogPos(value): jog_joint(7, value, "pos")
    def J8jogNeg(value): jog_joint(8, value, "neg")
    def J8jogPos(value): jog_joint(8, value, "pos")
    def J9jogNeg(value): jog_joint(9, value, "neg")
    def J9jogPos(value): jog_joint(9, value, "pos")

    def LiveJointJog(value):
        global xboxUse

        # Update status labels
        almStatusLab.configure(text="SYSTEM READY", text_color="green")
        almStatusLab2.configure(text="SYSTEM READY", text_color="green")

        # Check and adjust speed settings
        checkSpeedVals()
        speedtype = speedOption.get()

        # Ensure speed type is valid and set speed prefix and value
        speedPrefix = "Sp" if speedtype in ["Percent", "mm per Sec"] else "Ss"
        if speedtype == "mm per Sec":
            speedOption.set("Percent")
            speedEntryField.delete(0, 'end')
            speedEntryField.insert(0, "50")

        # Retrieve current values from entry fields
        Speed = speedEntryField.get()
        ACCspd = ACCspeedField.get()
        DECspd = DECspeedField.get()
        ACCramp = ACCrampField.get()
        LoopMode = "".join(str(joint.get()) for joint in [
            J1OpenLoopStat, J2OpenLoopStat, J3OpenLoopStat,
            J4OpenLoopStat, J5OpenLoopStat, J6OpenLoopStat
        ])

        # Construct and send command
        command = f"LJV{value}{speedPrefix}{Speed}Ac{ACCspd}Dc{DECspd}Rm{ACCramp}W{WC}Lm{LoopMode}\n"
        ser.write(command.encode())
        
        # Update command sent field and read response
        cmdSentEntryField.delete(0, 'end')
        cmdSentEntryField.insert(0, command)
        ser.flushInput()
        time.sleep(0.1)
        ser.read()

    def LiveCarJog(value):
        global xboxUse

        # Update status labels
        almStatusLab.configure(text="SYSTEM READY", text_color="green")
        almStatusLab2.configure(text="SYSTEM READY", text_color="green")

        # Check and adjust speed settings
        checkSpeedVals()
        speedtype = speedOption.get()

        # Ensure speed type is valid and set speed prefix and value
        speedPrefix = "Sp" if speedtype in ["Percent", "mm per Sec"] else "Ss"
        if speedtype == "mm per Sec":
            speedOption.set("Percent")
            speedEntryField.delete(0, 'end')
            speedEntryField.insert(0, "50")

        # Retrieve current values from entry fields
        Speed = speedEntryField.get()
        ACCspd = ACCspeedField.get()
        DECspd = DECspeedField.get()
        ACCramp = ACCrampField.get()
        LoopMode = "".join(str(joint.get()) for joint in [
            J1OpenLoopStat, J2OpenLoopStat, J3OpenLoopStat,
            J4OpenLoopStat, J5OpenLoopStat, J6OpenLoopStat
        ])

        # Construct and send command
        command = f"LCV{value}{speedPrefix}{Speed}Ac{ACCspd}Dc{DECspd}Rm{ACCramp}W{WC}Lm{LoopMode}\n"
        ser.write(command.encode())
        
        # Update command sent field and read response
        cmdSentEntryField.delete(0, 'end')
        cmdSentEntryField.insert(0, command)
        ser.flushInput()
        time.sleep(0.1)
        ser.read()

    def LiveToolJog(value):
        global xboxUse

        # Update status labels
        almStatusLab.configure(text="SYSTEM READY", text_color="green")
        almStatusLab2.configure(text="SYSTEM READY", text_color="green")

        # Check and adjust speed settings
        checkSpeedVals()
        speedtype = speedOption.get()

        # Ensure speed type is valid and set speed prefix and value
        speedPrefix = "Sp" if speedtype in ["Percent", "mm per Sec"] else "Ss"
        if speedtype == "mm per Sec":
            speedOption.set("Percent")
            speedEntryField.delete(0, 'end')
            speedEntryField.insert(0, "50")

        # Retrieve current values from entry fields
        Speed = speedEntryField.get()
        ACCspd = ACCspeedField.get()
        DECspd = DECspeedField.get()
        ACCramp = ACCrampField.get()
        LoopMode = "".join(str(joint.get()) for joint in [
            J1OpenLoopStat, J2OpenLoopStat, J3OpenLoopStat,
            J4OpenLoopStat, J5OpenLoopStat, J6OpenLoopStat
        ])

        # Construct and send command
        command = f"LTV{value}{speedPrefix}{Speed}Ac{ACCspd}Dc{DECspd}Rm{ACCramp}W{WC}Lm{LoopMode}\n"
        ser.write(command.encode())
        
        # Update command sent field and read response
        cmdSentEntryField.delete(0, 'end')
        cmdSentEntryField.insert(0, command)
        ser.flushInput()
        time.sleep(0.1)
        ser.read()

    def StopJog():
        command = "S\n"
        if int(IncJogStat.get()) == 0:
            ser.write(command.encode())
            ser.flushInput()
            time.sleep(0.1)
            
            # Read and handle response
            response = ser.readline().decode('utf-8').strip()
            if response.startswith('E'):
                ErrorHandler(response)
            else:
                displayPosition(response)

    def jog_joint_command(joint_index, value, direction):

        # Helper function to handle jogging for a specific joint in a given direction.
        
        """
        Args:
            joint_index (int): The index of the joint to jog (7, 8, or 9).
            value (float): The amount to adjust the joint position.
            direction (str): Either "pos" or "neg" to indicate the direction of jog.
        """
        global xboxUse

        checkSpeedVals()
        if xboxUse != 1:
            almStatusLab.config(text="SYSTEM READY", style="OK.TLabel")
            almStatusLab2.config(text="SYSTEM READY", style="OK.TLabel")

        # Determine speed prefix and set default speed if needed
        speedtype = speedOption.get()
        if speedtype == "mm per Sec":
            speedPrefix = "Sp"
            speedEntryField.delete(0, 'end')
            speedEntryField.insert(0, "50")
        elif speedtype == "Seconds":
            speedPrefix = "Ss"
        else:
            speedPrefix = "Sp"

        Speed = speedEntryField.get()
        ACCspd = ACCspeedField.get()
        DECspd = DECspeedField.get()
        ACCramp = ACCrampField.get()
        LoopMode = "".join(str(stat.get()) for stat in [J1OpenLoopStat, J2OpenLoopStat, J3OpenLoopStat, J4OpenLoopStat, J5OpenLoopStat, J6OpenLoopStat])

        # Adjust joint position based on direction
        joint_positions = [J1AngCur, J2AngCur, J3AngCur, J4AngCur, J5AngCur, J6AngCur, J7PosCur, J8PosCur, J9PosCur]
        if direction == "neg":
            joint_positions[joint_index - 1] = str(float(joint_positions[joint_index - 1]) - value)
        else:
            joint_positions[joint_index - 1] = str(float(joint_positions[joint_index - 1]) + value)

        # Build command string
        command = f"RJ" + "".join(f"{axis}{pos}" for axis, pos in zip("ABCDEF", joint_positions[:6])) + \
                f"J7{joint_positions[6]}J8{joint_positions[7]}J9{joint_positions[8]}{speedPrefix}{Speed}" + \
                f"Ac{ACCspd}Dc{DECspd}Rm{ACCramp}W{WC}Lm{LoopMode}\n"

        # Send command to serial
        ser.write(command.encode())
        cmdSentEntryField.delete(0, 'end')
        cmdSentEntryField.insert(0, command)
        ser.flushInput()
        time.sleep(.1)

        # Process response
        response = str(ser.readline().strip(), 'utf-8')
        if response.startswith('E'):
            ErrorHandler(response)
        else:
            displayPosition(response)

    # Define jog functions for J7, J8, and J9 using the helper function
    def J7jogNeg(value): jog_joint_command(7, value, "neg")
    def J7jogPos(value): jog_joint_command(7, value, "pos")
    def J8jogNeg(value): jog_joint_command(8, value, "neg")
    def J8jogPos(value): jog_joint_command(8, value, "pos")
    def J9jogNeg(value): jog_joint_command(9, value, "neg")
    def J9jogPos(value): jog_joint_command(9, value, "pos")

    def jog_neg_with_command(axis, value):
        # Get speed prefix based on speed option
        speedtype = speedOption.get()
        speedPrefix = ""
        if speedtype == "mm per Sec":
            speedPrefix = "Sm"
        elif speedtype == "Seconds":
            speedPrefix = "Ss"
        elif speedtype == "Percent":
            speedPrefix = "Sp"

        # Retrieve necessary speed values
        Speed = speedEntryField.get()
        ACCspd = ACCspeedField.get()
        DECspd = DECspeedField.get()
        ACCramp = ACCrampField.get()
        j7Val = str(J7PosCur)
        j8Val = str(J8PosCur)
        j9Val = str(J9PosCur)
        
        # Compile loop mode status for each joint
        LoopMode = ''.join([
            str(J1OpenLoopStat.get()), str(J2OpenLoopStat.get()), str(J3OpenLoopStat.get()),
            str(J4OpenLoopStat.get()), str(J5OpenLoopStat.get()), str(J6OpenLoopStat.get())
        ])
        
        # Check if Xbox controller is in use
        global xboxUse
        checkSpeedVals()
        if xboxUse != 1:
            almStatusLab.config(text="SYSTEM READY", style="OK.TLabel")
            almStatusLab2.config(text="SYSTEM READY", style="OK.TLabel")
        
        # Calculate new positions based on axis and value
        xVal = XcurPos if axis != 'X' else str(float(XcurPos) - value)
        yVal = YcurPos if axis != 'Y' else str(float(YcurPos) - value)
        zVal = ZcurPos if axis != 'Z' else str(float(ZcurPos) - value)
        rzVal = RzcurPos if axis != 'Rz' else str(float(RzcurPos) - value)
        ryVal = RycurPos if axis != 'Ry' else str(float(RycurPos) - value)
        rxVal = RxcurPos if axis != 'Rx' else str(float(RxcurPos) - value)

        # Generate command string
        command = (
            f"MJX{xVal}Y{yVal}Z{zVal}Rz{rzVal}Ry{ryVal}Rx{rxVal}J7{j7Val}J8{j8Val}"
            f"J9{j9Val}{speedPrefix}{Speed}Ac{ACCspd}Dc{DECspd}Rm{ACCramp}W{WC}Lm{LoopMode}\n"
        )

        # Send command and handle response
        ser.write(command.encode())
        cmdSentEntryField.delete(0, 'end')
        cmdSentEntryField.insert(0, command)
        ser.flushInput()
        time.sleep(0.1)
        response = str(ser.readline().strip(), 'utf-8')
        if response.startswith('E'):
            ErrorHandler(response)
        else:
            displayPosition(response)

    # Updated jog functions to use the consolidated function
    def XjogNeg(value):
        jog_neg_with_command('X', value)
    def YjogNeg(value):
        jog_neg_with_command('Y', value)
    def ZjogNeg(value):
        jog_neg_with_command('Z', value)
    def RxjogNeg(value):
        jog_neg_with_command('Rx', value)
    def RyjogNeg(value):
        jog_neg_with_command('Ry', value)
    def RzjogNeg(value):
        jog_neg_with_command('Rz', value)

    def jog_pos_with_command(axis, value):
        # Get speed prefix based on speed option
        speedtype = speedOption.get()
        speedPrefix = ""
        if speedtype == "mm per Sec":
            speedPrefix = "Sm"
        elif speedtype == "Seconds":
            speedPrefix = "Ss"
        elif speedtype == "Percent":
            speedPrefix = "Sp"

        # Retrieve necessary speed values
        Speed = speedEntryField.get()
        ACCspd = ACCspeedField.get()
        DECspd = DECspeedField.get()
        ACCramp = ACCrampField.get()
        j7Val = str(J7PosCur)
        j8Val = str(J8PosCur)
        j9Val = str(J9PosCur)
        
        # Compile loop mode status for each joint
        LoopMode = ''.join([
            str(J1OpenLoopStat.get()), str(J2OpenLoopStat.get()), str(J3OpenLoopStat.get()),
            str(J4OpenLoopStat.get()), str(J5OpenLoopStat.get()), str(J6OpenLoopStat.get())
        ])
        
        # Check if Xbox controller is in use
        global xboxUse
        checkSpeedVals()
        if xboxUse != 1:
            almStatusLab.config(text="SYSTEM READY", style="OK.TLabel")
            almStatusLab2.config(text="SYSTEM READY", style="OK.TLabel")
        
        # Calculate new positions based on axis and value for positive jog
        xVal = XcurPos if axis != 'X' else str(float(XcurPos) + value)
        yVal = YcurPos if axis != 'Y' else str(float(YcurPos) + value)
        zVal = ZcurPos if axis != 'Z' else str(float(ZcurPos) + value)
        rzVal = RzcurPos if axis != 'Rz' else str(float(RzcurPos) + value)
        ryVal = RycurPos if axis != 'Ry' else str(float(RycurPos) + value)
        rxVal = RxcurPos if axis != 'Rx' else str(float(RxcurPos) + value)

        # Generate command string
        command = (
            f"MJX{xVal}Y{yVal}Z{zVal}Rz{rzVal}Ry{ryVal}Rx{rxVal}J7{j7Val}J8{j8Val}"
            f"J9{j9Val}{speedPrefix}{Speed}Ac{ACCspd}Dc{DECspd}Rm{ACCramp}W{WC}Lm{LoopMode}\n"
        )

        # Send command and handle response
        ser.write(command.encode())
        cmdSentEntryField.delete(0, 'end')
        cmdSentEntryField.insert(0, command)
        ser.flushInput()
        time.sleep(0.1)
        response = str(ser.readline().strip(), 'utf-8')
        if response.startswith('E'):
            ErrorHandler(response)
        else:
            displayPosition(response)

    # Updated jog functions to use the consolidated function for positive jogs
    def XjogPos(value):
        jog_pos_with_command('X', value)
    def YjogPos(value):
        jog_pos_with_command('Y', value)
    def ZjogPos(value):
        jog_pos_with_command('Z', value)
    def RxjogPos(value):
        jog_pos_with_command('Rx', value)
    def RyjogPos(value):
        jog_pos_with_command('Ry', value)
    def RzjogPos(value):
        jog_pos_with_command('Rz', value)

    def execute_t_jog_neg(axis_prefix, value):
        # Check and set system readiness
        global xboxUse
        checkSpeedVals()
        if xboxUse != 1:
            almStatusLab.config(text="SYSTEM READY", style="OK.TLabel")
            almStatusLab2.config(text="SYSTEM READY", style="OK.TLabel")

        # Handle speed settings, restricting "mm per Sec" if needed
        speedtype = speedOption.get()
        if speedtype == "mm per Sec":
            speedMenu = OptionMenu(tab1, speedOption, "Percent", "Percent", "Seconds", "mm per Sec")
            speedPrefix = "Ss"
            speedEntryField.delete(0, 'end')
            speedEntryField.insert(0, "50")
        elif speedtype == "Seconds":
            speedPrefix = "Ss"
        elif speedtype == "Percent":
            speedPrefix = "Sp"
        else:
            speedPrefix = ""

        # Collect other parameters
        Speed = speedEntryField.get()
        ACCspd = ACCspeedField.get()
        DECspd = DECspeedField.get()
        ACCramp = ACCrampField.get()

        # Compile loop mode
        LoopMode = ''.join([
            str(J1OpenLoopStat.get()), str(J2OpenLoopStat.get()), str(J3OpenLoopStat.get()),
            str(J4OpenLoopStat.get()), str(J5OpenLoopStat.get()), str(J6OpenLoopStat.get())
        ])

        # Construct the command for the specific T-axis jog
        command = (
            f"JT{axis_prefix}1{value}{speedPrefix}{Speed}G{ACCspd}H{DECspd}"
            f"I{ACCramp}Lm{LoopMode}\n"
        )

        # Send the command and handle response
        ser.write(command.encode())
        cmdSentEntryField.delete(0, 'end')
        cmdSentEntryField.insert(0, command)
        ser.flushInput()
        time.sleep(0.1)
        response = str(ser.readline().strip(), 'utf-8')
        if response.startswith('E'):
            ErrorHandler(response)
        else:
            displayPosition(response)

    # Individual jog functions now call execute_t_jog_neg with their specific axis_prefix
    def TXjogNeg(value):
        execute_t_jog_neg('X', value)
    def TYjogNeg(value):
        execute_t_jog_neg('Y', value)
    def TZjogNeg(value):
        execute_t_jog_neg('Z', value)
    def TRxjogNeg(value):
        execute_t_jog_neg('W', value)
    def TRyjogNeg(value):
        execute_t_jog_neg('P', value)
    def TRzjogNeg(value):
        execute_t_jog_neg('R', value)

    def execute_t_jog_pos(axis_prefix, value):
        # Check and set system readiness
        global xboxUse
        checkSpeedVals()
        if xboxUse != 1:
            almStatusLab.config(text="SYSTEM READY", style="OK.TLabel")
            almStatusLab2.config(text="SYSTEM READY", style="OK.TLabel")

        # Handle speed settings, restricting "mm per Sec" if needed
        speedtype = speedOption.get()
        if speedtype == "mm per Sec":
            speedMenu = OptionMenu(tab1, speedOption, "Percent", "Percent", "Seconds", "mm per Sec")
            speedPrefix = "Ss"
            speedEntryField.delete(0, 'end')
            speedEntryField.insert(0, "50")
        elif speedtype == "Seconds":
            speedPrefix = "Ss"
        elif speedtype == "Percent":
            speedPrefix = "Sp"
        else:
            speedPrefix = ""

        # Collect other parameters
        Speed = speedEntryField.get()
        ACCspd = ACCspeedField.get()
        DECspd = DECspeedField.get()
        ACCramp = ACCrampField.get()

        # Compile loop mode
        LoopMode = ''.join([
            str(J1OpenLoopStat.get()), str(J2OpenLoopStat.get()), str(J3OpenLoopStat.get()),
            str(J4OpenLoopStat.get()), str(J5OpenLoopStat.get()), str(J6OpenLoopStat.get())
        ])

        # Construct the command for the specific T-axis positive jog
        command = (
            f"JT{axis_prefix}1+{value}{speedPrefix}{Speed}G{ACCspd}H{DECspd}"
            f"I{ACCramp}Lm{LoopMode}\n"
        )

        # Send the command and handle response
        ser.write(command.encode())
        cmdSentEntryField.delete(0, 'end')
        cmdSentEntryField.insert(0, command)
        ser.flushInput()
        time.sleep(0.1)
        response = str(ser.readline().strip(), 'utf-8')
        if response.startswith('E'):
            ErrorHandler(response)
        else:
            displayPosition(response)

    # Individual jog functions now call execute_t_jog_pos with their specific axis_prefix
    def TXjogPos(value):
        execute_t_jog_pos('X', value)
    def TYjogPos(value):
        execute_t_jog_pos('Y', value)
    def TZjogPos(value):
        execute_t_jog_pos('Z', value)
    def TRxjogPos(value):
        execute_t_jog_pos('W', value)
    def TRyjogPos(value):
        execute_t_jog_pos('P', value)
    def TRzjogPos(value):
        execute_t_jog_pos('R', value)

    # Teach defs #

    def teachInsertBelSelected():
        global XcurPos, YcurPos, ZcurPos, RxcurPos, RycurPos, RzcurPos, WC, J7PosCur

        def get_selected_row():
            try:
                sel_row = tab1.progView.curselection()[0] + 1
            except:
                last = tab1.progView.index('end')
                sel_row = last
                tab1.progView.select_set(sel_row)
            return sel_row

        def determine_speed_prefix(speed_type):
            if speed_type == "Seconds":
                return "Ss"
            elif speed_type == "mm per Sec":
                return "Sm"
            elif speed_type == "Percent":
                return "Sp"
            return ""

        def insert_to_view_and_save(new_position, sel_row):
            tab1.progView.insert(sel_row, bytes(new_position + '\n', 'utf-8'))
            tab1.progView.selection_clear(0, 'end')
            tab1.progView.select_set(sel_row)
            items = tab1.progView.get(0, 'end')
            file_path = path.relpath(ProgEntryField.get())
            with open(file_path, 'w', encoding='utf-8') as f:
                for item in items:
                    f.write(str(item.strip(), encoding='utf-8'))
                    f.write('\n')

        # Main function code starts here

        check_speed_vals()
        sel_row = get_selected_row()

        Speed = speedEntryField.get()
        speed_type = speedOption.get()
        speed_prefix = determine_speed_prefix(speed_type)

        ACCspd = ACCspeedField.get()
        DECspd = DECspeedField.get()
        ACCramp = ACCrampField.get()
        Rounding = roundEntryField.get()
        movetype = options.get()

        # Handle each movetype case and call insert_to_view_and_save accordingly
        if movetype in ["OFF J", "Move Vis", "Move J", "Move L", "Move R", "Move A Mid", "Move A End", "Move C Center"]:
            new_position = (
                f"{movetype} [*] X {XcurPos} Y {YcurPos} Z {ZcurPos} Rz {RzcurPos} "
                f"Ry {RycurPos} Rx {RxcurPos} J7 {J7PosCur} J8 {J8PosCur} J9 {J9PosCur} "
                f"{speed_prefix} {Speed} Ac {ACCspd} Dc {DECspd} Rm {ACCramp} $ {WC}"
            )
            insert_to_view_and_save(new_position, sel_row)

        elif movetype == "Move PR":
            new_position = (
                f"{movetype} [ PR: {SavePosEntryField.get()} ] [*] J7 {J7PosCur} J8 {J8PosCur} "
                f"J9 {J9PosCur} {speed_prefix} {Speed} Ac {ACCspd} Dc {DECspd} Rm {ACCramp} $ {WC}"
            )
            insert_to_view_and_save(new_position, sel_row)

        elif movetype == "OFF PR ":
            new_position = (
                f"{movetype} [ PR: {SavePosEntryField.get()} ] offs [ *PR: {int(SavePosEntryField.get()) + 1} ] "
                f"[*] J7 {J7PosCur} J8 {J8PosCur} J9 {J9PosCur} {speed_prefix} {Speed} Ac {ACCspd} Dc {DECspd} Rm {ACCramp} $ {WC}"
            )
            insert_to_view_and_save(new_position, sel_row)

        elif movetype == "Move C Start":
            new_position = f"{movetype} [*] X {XcurPos} Y {YcurPos} Z {ZcurPos}"
            insert_to_view_and_save(new_position, sel_row)

        elif movetype == "Teach PR":
            PR = str(SavePosEntryField.get())
            elements = [
                f"Position Register {PR} Element 6 = {RxcurPos}",
                f"Position Register {PR} Element 5 = {RycurPos}",
                f"Position Register {PR} Element 4 = {RzcurPos}",
                f"Position Register {PR} Element 3 = {ZcurPos}",
                f"Position Register {PR} Element 2 = {YcurPos}",
                f"Position Register {PR} Element 1 = {XcurPos}"
            ]
            for element in elements:
                tab1.progView.insert(sel_row, bytes(element + '\n', 'utf-8'))
                sel_row += 1
            insert_to_view_and_save("", sel_row)
    
    def teachReplaceSelected():
        try:
            deleteitem()
            selRow = tab1.progView.curselection()[0]
            tab1.progView.select_set(selRow - 1)
        except IndexError:
            selRow = tab1.progView.index('end')
            tab1.progView.select_set(selRow)

        teachInsertBelSelected()

    # Program function defs #

    def deleteitem():
        try:
            # Get the currently selected row index
            selRow = tab1.progView.curselection()[0]
            
            # Delete the selected item and clear any selection
            tab1.progView.delete(selRow)
            tab1.progView.selection_clear(0, END)
            
            # Re-select the current row if possible
            tab1.progView.select_set(min(selRow, tab1.progView.index('end') - 1))

            # Save the updated list of items back to the file
            items = tab1.progView.get(0, END)
            file_path = path.relpath(ProgEntryField.get())
            with open(file_path, 'w', encoding='utf-8') as f:
                for item in items:
                    f.write(str(item.strip(), encoding='utf-8') + '\n')
        except IndexError:
            # Handle the case when no selection is available
            pass

    def manInsItem():
        try:
            sel_row = tab1.prog_view.curselection()[0]
            sel_row += 1
        except IndexError:  # handle specific exception for no selection
            last = tab1.prog_view.size() - 1
            sel_row = last
            tab1.prog_view.select_set(sel_row)
        
        # Insert the item and clear previous selections
        tab1.prog_view.insert(sel_row, bytes(man_entry_field.get() + '\n', 'utf-8'))
        tab1.prog_view.selection_clear(0, ctk.END)
        tab1.prog_view.select_set(sel_row)
        
        # Update current row entry
        cur_row_entry_field.delete(0, 'end')
        cur_row_entry_field.insert(0, sel_row)
        
        # Set item color (in CTk, you may need to manage text color in other ways)
        tab1.prog_view.itemconfig(sel_row, {'foreground': 'darkgreen'})
        
        # Write updated list to file
        items = tab1.prog_view.get(0, ctk.END)
        file_path = os.path.relpath(prog_entry_field.get())
        with open(file_path, 'w', encoding='utf-8') as file:
            for item in items:
                file.write(item.decode('utf-8').strip() + '\n')

    def manReplItem():
        # Get the selected row
        try:
            sel_row = tab1.prog_view.curselection()[0]
        except IndexError:  # Handle case where no row is selected
            return
        
        # Delete and replace the item at selected row
        tab1.prog_view.delete(sel_row)
        tab1.prog_view.insert(sel_row, bytes(man_entry_field.get() + '\n', 'utf-8'))
        
        # Update selection and clear previous selections
        tab1.prog_view.selection_clear(0, ctk.END)
        tab1.prog_view.select_set(sel_row)
        
        # Update item color (CTk might need alternative styling if not directly supported)
        tab1.prog_view.itemconfig(sel_row, {'foreground': 'darkgreen'})
        
        # Write updated list to file
        items = tab1.prog_view.get(0, ctk.END)
        file_path = os.path.relpath(prog_entry_field.get())
        with open(file_path, 'w', encoding='utf-8') as file:
            for item in items:
                file.write(item.decode('utf-8').strip() + '\n')

    def waitTime():
        try:
            # Get the selected row and increment by 1 to insert below the current selection
            sel_row = tab1.prog_view.curselection()[0] + 1
        except IndexError:
            # If no selection, set sel_row to the last position
            sel_row = tab1.prog_view.size()
        
        # Prepare the new "Wait Time" text
        seconds = wait_time_entry_field.get()
        new_time = f"Wait Time = {seconds}"
        
        # Insert new item in the list
        tab1.prog_view.insert(sel_row, bytes(new_time + '\n', 'utf-8'))
        tab1.prog_view.selection_clear(0, ctk.END)
        tab1.prog_view.select_set(sel_row)
        
        # Write the updated list to the file
        items = tab1.prog_view.get(0, ctk.END)
        file_path = os.path.relpath(prog_entry_field.get())
        with open(file_path, 'w', encoding='utf-8') as file:
            for item in items:
                file.write(item.decode('utf-8').strip() + '\n')

    def waitInputOn():
        try:
            # Get the selected row and increment by 1 to insert below the current selection
            sel_row = tab1.prog_view.curselection()[0] + 1
        except IndexError:
            # If no selection, set sel_row to the last position
            sel_row = tab1.prog_view.size()
        
        # Prepare the "Wait Input On" text
        input_value = wait_input_entry_field.get()
        new_input = f"Wait Input On = {input_value}"
        
        # Insert new item in the list
        tab1.prog_view.insert(sel_row, bytes(new_input + '\n', 'utf-8'))
        tab1.prog_view.selection_clear(0, ctk.END)
        tab1.prog_view.select_set(sel_row)
        
        # Write the updated list to the file
        items = tab1.prog_view.get(0, ctk.END)
        file_path = os.path.relpath(prog_entry_field.get())
        with open(file_path, 'w', encoding='utf-8') as file:
            for item in items:
                file.write(item.decode('utf-8').strip() + '\n')

    def waitInputOff():
        try:
            # Get the selected row and increment by 1 to insert below the current selection
            sel_row = tab1.prog_view.curselection()[0] + 1
        except IndexError:
            # If no selection, set sel_row to the last position
            sel_row = tab1.prog_view.size()
        
        # Prepare the "Wait Off Input" text
        input_value = wait_input_off_entry_field.get()
        new_input = f"Wait Off Input = {input_value}"
        
        # Insert new item in the list
        tab1.prog_view.insert(sel_row, bytes(new_input + '\n', 'utf-8'))
        tab1.prog_view.selection_clear(0, ctk.END)
        tab1.prog_view.select_set(sel_row)
        
        # Write the updated list to the file
        items = tab1.prog_view.get(0, ctk.END)
        file_path = os.path.relpath(prog_entry_field.get())
        with open(file_path, 'w', encoding='utf-8') as file:
            for item in items:
                file.write(item.decode('utf-8').strip() + '\n')

    def setOutputOn():
        try:
            # Get the selected row and increment by 1 to insert below the current selection
            sel_row = tab1.prog_view.curselection()[0] + 1
        except IndexError:
            # If no selection, set sel_row to the last position
            sel_row = tab1.prog_view.size()
        
        # Prepare the "Out On" text
        output_value = output_on_entry_field.get()
        new_output = f"Out On = {output_value}"
        
        # Insert new item in the list
        tab1.prog_view.insert(sel_row, bytes(new_output + '\n', 'utf-8'))
        tab1.prog_view.selection_clear(0, ctk.END)
        tab1.prog_view.select_set(sel_row)
        
        # Write the updated list to the file
        items = tab1.prog_view.get(0, ctk.END)
        file_path = os.path.relpath(prog_entry_field.get())
        with open(file_path, 'w', encoding='utf-8') as file:
            for item in items:
                file.write(item.decode('utf-8').strip() + '\n')

    def setOutputOff():
        try:
            # Get the selected row and increment by 1 to insert below the current selection
            sel_row = tab1.prog_view.curselection()[0] + 1
        except IndexError:
            # If no selection, set sel_row to the last position
            sel_row = tab1.prog_view.size()
        
        # Prepare the "Out Off" text
        output_value = output_off_entry_field.get()
        new_output = f"Out Off = {output_value}"
        
        # Insert new item in the list
        tab1.prog_view.insert(sel_row, bytes(new_output + '\n', 'utf-8'))
        tab1.prog_view.selection_clear(0, ctk.END)
        tab1.prog_view.select_set(sel_row)
        
        # Write the updated list to the file
        items = tab1.prog_view.get(0, ctk.END)
        file_path = os.path.relpath(prog_entry_field.get())
        with open(file_path, 'w', encoding='utf-8') as file:
            for item in items:
                file.write(item.decode('utf-8').strip() + '\n')

    def tabNumber():
        try:
            # Get the selected row and increment by 1 to insert below the current selection
            sel_row = tab1.prog_view.curselection()[0] + 1
        except IndexError:
            # If no selection, set sel_row to the last position
            sel_row = tab1.prog_view.size()
        
        # Prepare the "Tab Number" text
        tab_num = tab_num_entry_field.get()
        tab_insert = f"Tab Number {tab_num}"
        
        # Insert new item in the list
        tab1.prog_view.insert(sel_row, bytes(tab_insert + '\n', 'utf-8'))
        tab1.prog_view.selection_clear(0, ctk.END)
        tab1.prog_view.select_set(sel_row)
        
        # Write the updated list to the file
        items = tab1.prog_view.get(0, ctk.END)
        file_path = os.path.relpath(prog_entry_field.get())
        with open(file_path, 'w', encoding='utf-8') as file:
            for item in items:
                file.write(item.decode('utf-8').strip() + '\n')

    def jumpTab():
        try:
            # Get the selected row and increment by 1 to insert below the current selection
            sel_row = tab1.prog_view.curselection()[0] + 1
        except IndexError:
            # If no selection, set sel_row to the last position
            sel_row = tab1.prog_view.size()
        
        # Prepare the "Jump Tab" text
        tab_num = jump_tab_entry_field.get()
        tab_jump_text = f"Jump Tab-{tab_num}"
        
        # Insert new item in the list
        tab1.prog_view.insert(sel_row, bytes(tab_jump_text + '\n', 'utf-8'))
        tab1.prog_view.selection_clear(0, ctk.END)
        tab1.prog_view.select_set(sel_row)
        
        # Write the updated list to the file
        items = tab1.prog_view.get(0, ctk.END)
        file_path = os.path.relpath(prog_entry_field.get())
        with open(file_path, 'w', encoding='utf-8') as file:
            for item in items:
                file.write(item.decode('utf-8').strip() + '\n')

    def cameraOn():
        try:
            # Get the selected row and increment by 1 to insert below the current selection
            selRow = tab1.progView.curselection()[0] + 1
        except IndexError:
            # If no selection, set selRow to the last position
            selRow = tab1.progView.size()
        
        # Insert "Cam On" text into the list
        value = "Cam On"
        tab1.progView.insert(selRow, bytes(value + '\n', 'utf-8'))
        tab1.progView.selection_clear(0, ctk.END)
        tab1.progView.select_set(selRow)
        
        # Write the updated list to the file
        items = tab1.progView.get(0, ctk.END)
        file_path = os.path.relpath(ProgEntryField.get())
        with open(file_path, 'w', encoding='utf-8') as f:
            for item in items:
                f.write(item.decode('utf-8').strip() + '\n')

    def cameraOff():
        try:
            selRow = tab1.progView.curselection()[0]
            selRow += 1
        except IndexError:
            last = tab1.progView.size() - 1
            selRow = last
            tab1.progView.select_set(selRow)
        
        value = "Cam Off"
        tab1.progView.insert(selRow, bytes(value + '\n', 'utf-8'))
        tab1.progView.selection_clear(0, ctk.END)
        tab1.progView.select_set(selRow)
        
        items = tab1.progView.get(0, ctk.END)
        file_path = os.path.relpath(ProgEntryField.get())
        
        with open(file_path, 'w', encoding='utf-8') as f:
            for item in items:
                # Strip and decode each item from bytes back to a string for consistent UTF-8 encoding in the file
                f.write(item.strip().decode('utf-8'))
                f.write('\n')

    def IfCMDInsert():
        localErrorFlag = False
        try:
            selRow = tab1.progView.curselection()[0]
            selRow += 1
        except IndexError:
            last = tab1.progView.size() - 1
            selRow = last
            tab1.progView.select_set(selRow)

        option = iFoption.get()
        selection = iFselection.get()
        variable = IfVarEntryField.get()
        
        if not variable:
            localErrorFlag = True
            almStatusLab.config(text="Please enter an input, register number or COM Port", style="Alarm.TLabel")
            
        inputVal = IfInputEntryField.get()
        destVal = IfDestEntryField.get()
        prefix = ""

        if option == "Input":
            if inputVal in ["0", "1"]:
                prefix = f"If Input # {variable} = {inputVal} :"
            else:
                localErrorFlag = True
                almStatusLab.config(text="Please enter a 1 or 0 for the = value", style="Alarm.TLabel")
        
        elif option == "Register":
            if not inputVal:
                localErrorFlag = True
                almStatusLab.config(text="Please enter a register number", style="Alarm.TLabel")
            prefix = f"If Register # {variable} = {inputVal} :"

        elif option == "COM Device":
            if not inputVal:
                localErrorFlag = True
                almStatusLab.config(text="Please enter expected COM device input", style="Alarm.TLabel")
            prefix = f"If COM Device # {variable} = {inputVal} :"
        
        if selection == "Call Prog":
            if not destVal:
                localErrorFlag = True
                almStatusLab.config(text="Please enter a program name", style="Alarm.TLabel")
            value = f"{prefix} Call Prog {destVal}"
        
        elif selection == "Jump Tab":
            if not destVal:
                localErrorFlag = True
                almStatusLab.config(text="Please enter a destination tab", style="Alarm.TLabel")
            value = f"{prefix} Jump to Tab {destVal}"
        
        if not localErrorFlag:
            tab1.progView.insert(selRow, bytes(value + '\n', 'utf-8'))
            tab1.progView.selection_clear(0, ctk.END)
            tab1.progView.select_set(selRow)
            
            items = tab1.progView.get(0, ctk.END)
            file_path = os.path.relpath(ProgEntryField.get())
            
            with open(file_path, 'w', encoding='utf-8') as f:
                for item in items:
                    f.write(item.strip().decode('utf-8'))
                    f.write('\n')

    def ReadAuxCom():
        try:
            selRow = tab1.progView.curselection()[0]
            selRow += 1
        except IndexError:
            last = tab1.progView.size() - 1
            selRow = last
            tab1.progView.select_set(selRow)

        comNum = auxPortEntryField.get()
        comChar = auxCharEntryField.get()
        servoins = f"Read COM # {comNum} Char: {comChar}"
        
        tab1.progView.insert(selRow, bytes(servoins + '\n', 'utf-8'))
        tab1.progView.selection_clear(0, ctk.END)
        tab1.progView.select_set(selRow)
        
        items = tab1.progView.get(0, ctk.END)
        file_path = os.path.relpath(ProgEntryField.get())
        
        with open(file_path, 'w', encoding='utf-8') as f:
            for item in items:
                f.write(item.strip().decode('utf-8'))
                f.write('\n')

    def TestAuxCom():
        global ser3
        try:
            port = f"COM{com3PortEntryField.get()}"
            baud = 115200
            ser3 = serial.Serial(port, baud, timeout=5)
            ser3.flushInput()
        except serial.SerialException:
            Curtime = datetime.datetime.now().strftime("%B %d %Y - %I:%M%p")
            error_message = f"{Curtime} - UNABLE TO ESTABLISH COMMUNICATIONS WITH SERIAL DEVICE"
            tab8.ElogView.insert(ctk.END, error_message)
            
            # Save error log
            value = tab8.ElogView.get(0, ctk.END)
            with open("ErrorLog", "wb") as f:
                pickle.dump(value, f)
            return  # Exit if connection fails

        numChar = int(com3charPortEntryField.get())
        response = ser3.read(numChar).strip().decode('utf-8')
        
        # Update output field
        com3outPortEntryField.delete(0, ctk.END)
        com3outPortEntryField.insert(0, response)

    def Servo():
        try:
            selRow = tab1.progView.curselection()[0]
            selRow += 1
        except IndexError:
            last = tab1.progView.size() - 1
            selRow = last
            tab1.progView.select_set(selRow)

        servoNum = servoNumEntryField.get()
        servoPos = servoPosEntryField.get()
        servoins = f"Servo number {servoNum} to position: {servoPos}"
        
        tab1.progView.insert(selRow, bytes(servoins + '\n', 'utf-8'))
        tab1.progView.selection_clear(0, ctk.END)
        tab1.progView.select_set(selRow)
        
        items = tab1.progView.get(0, ctk.END)
        file_path = os.path.relpath(ProgEntryField.get())
        
        with open(file_path, 'w', encoding='utf-8') as f:
            for item in items:
                f.write(item.strip().decode('utf-8'))
                f.write('\n')

    def loadProg():
        # Determine the folder based on whether the app is frozen (e.g., compiled with PyInstaller) or running as a script
        if getattr(sys, 'frozen', False):
            folder = os.path.dirname(sys.executable)
        else:
            folder = os.path.dirname(os.path.realpath(__file__))

        filetypes = (('Robot Program', '*.ar'), ("All Files", "*.*"))
        filename = fd.askopenfilename(title='Open File', initialdir=folder, filetypes=filetypes)
        
        if filename:
            name = os.path.basename(filename)
            ProgEntryField.delete(0, ctk.END)
            ProgEntryField.insert(0, name)
            
            # Clear the current content in progView and load the selected file
            tab1.progView.delete(0, ctk.END)
            
            with open(filename, "rb") as Prog:
                time.sleep(0.1)  # Optional sleep
                for item in Prog:
                    tab1.progView.insert(ctk.END, item)
            
            tab1.progView.pack()
            scrollbar.config(command=tab1.progView.yview)
            savePosData()

    def callProg(name):
        # Update the program entry field with the provided name
        ProgEntryField.delete(0, ctk.END)
        ProgEntryField.insert(0, name)
        
        # Clear the current content in progView
        tab1.progView.delete(0, ctk.END)
        
        # Open the file in binary mode and insert each line into progView
        with open(name, "rb") as Prog:
            time.sleep(0.1)  # Optional delay
            for item in Prog:
                tab1.progView.insert(ctk.END, item)
        
        tab1.progView.pack()
        scrollbar.config(command=tab1.progView.yview)

    def CreateProg():
        # Prompt user for a new program name
        user_input = simpledialog.askstring(title="New Program", prompt="New Program Name:")
        if not user_input:
            return  # Exit if the user cancels or provides no input
        
        file_path = f"{user_input}.ar"
        
        # Create a new file and write initial content
        with open(file_path, 'w', encoding='utf-8') as f:
            f.write("##BEGINNING OF PROGRAM##\n")
        
        # Update the program entry field with the new file path
        ProgEntryField.delete(0, ctk.END)
        ProgEntryField.insert(0, file_path)
        
        # Clear the current content in progView and load the newly created file
        tab1.progView.delete(0, ctk.END)
        
        with open(file_path, "rb") as Prog:
            time.sleep(0.1)  # Optional delay
            for item in Prog:
                tab1.progView.insert(ctk.END, item)
        
        tab1.progView.pack()
        scrollbar.config(command=tab1.progView.yview)
        savePosData()

    def insertCallProg():
        try:
            selRow = tab1.progView.curselection()[0]
            selRow += 1
        except IndexError:
            last = tab1.progView.size() - 1
            selRow = last
            tab1.progView.select_set(selRow)

        newProg = changeProgEntryField.get()
        changeProg = f"Call Program - {newProg}"
        
        # Ensure the program name has the correct extension
        if not changeProg.endswith(".ar"):
            changeProg += ".ar"
        
        # Insert the call program instruction
        tab1.progView.insert(selRow, bytes(changeProg + '\n', 'utf-8'))
        tab1.progView.selection_clear(0, ctk.END)
        tab1.progView.select_set(selRow)

        # Retrieve all items and save to file
        items = tab1.progView.get(0, ctk.END)
        file_path = os.path.relpath(ProgEntryField.get())
        
        with open(file_path, 'w', encoding='utf-8') as f:
            for item in items:
                f.write(item.strip().decode('utf-8'))
                f.write('\n')

    def insertGCprog():
        try:
            selRow = tab1.progView.curselection()[0]
            selRow += 1
        except IndexError:
            last = tab1.progView.size() - 1
            selRow = last
            tab1.progView.select_set(selRow)

        newProg = PlayGCEntryField.get()
        GCProg = f"Run Gcode Program - {newProg}"
        
        # Insert the Gcode program instruction
        tab1.progView.insert(selRow, bytes(GCProg + '\n', 'utf-8'))
        tab1.progView.selection_clear(0, ctk.END)
        tab1.progView.select_set(selRow)

        # Retrieve all items and save to file
        items = tab1.progView.get(0, ctk.END)
        file_path = os.path.relpath(ProgEntryField.get())
        
        with open(file_path, 'w', encoding='utf-8') as f:
            for item in items:
                f.write(item.strip().decode('utf-8'))
                f.write('\n')

    def insertReturn():
        try:
            selRow = tab1.progView.curselection()[0]
            selRow += 1
        except IndexError:
            last = tab1.progView.size() - 1
            selRow = last
            tab1.progView.select_set(selRow)

        value = "Return"
        
        # Insert the return instruction
        tab1.progView.insert(selRow, bytes(value + '\n', 'utf-8'))
        tab1.progView.selection_clear(0, ctk.END)
        tab1.progView.select_set(selRow)

        # Retrieve all items and save to file
        items = tab1.progView.get(0, ctk.END)
        file_path = os.path.relpath(ProgEntryField.get())
        
        with open(file_path, 'w', encoding='utf-8') as f:
            for item in items:
                f.write(item.strip().decode('utf-8'))
                f.write('\n')

    def openText():
        # Get the file path from the program entry field
        file_path = os.path.relpath(ProgEntryField.get())
        
        if os.path.exists(file_path):
            os.startfile(file_path)
        else:
            print(f"File not found: {file_path}")

    def reloadProg():
        file_path = os.path.relpath(ProgEntryField.get())
        
        # Update the program entry field with the reloaded file path
        ProgEntryField.delete(0, ctk.END)
        ProgEntryField.insert(0, file_path)
        
        # Clear the current content in progView and load the file
        tab1.progView.delete(0, ctk.END)
        
        with open(file_path, "rb") as Prog:
            time.sleep(0.1)  # Optional delay for smoother loading
            for item in Prog:
                tab1.progView.insert(ctk.END, item)
        
        # Refresh the view
        tab1.progView.pack()
        scrollbar.config(command=tab1.progView.yview)
        savePosData()

    def insertvisFind():
        global ZcurPos, RxcurPos, RycurPos, RzcurPos
        
        try:
            selRow = tab1.progView.curselection()[0]
            selRow += 1
        except IndexError:
            selRow = tab1.progView.size() - 1
            tab1.progView.select_set(selRow)

        # Get template and background color settings
        template = selectedTemplate.get() or "None_Selected.jpg"
        autoBGVal = int(autoBG.get())
        BGcolor = "(Auto)" if autoBGVal == 1 else VisBacColorEntryField.get()
        
        # Retrieve score, pass, and fail tab values
        score = VisScoreEntryField.get()
        passTab = visPassEntryField.get()
        failTab = visFailEntryField.get()
        
        # Construct the command string
        value = f"Vis Find - {template} - BGcolor {BGcolor} Score {score} Pass {passTab} Fail {failTab}"
        
        # Insert the command and update selection
        tab1.progView.insert(selRow, bytes(value + '\n', 'utf-8'))
        tab1.progView.selection_clear(0, ctk.END)
        tab1.progView.select_set(selRow)
        
        # Save all items to file
        items = tab1.progView.get(0, ctk.END)
        file_path = os.path.relpath(ProgEntryField.get())
        
        with open(file_path, 'w', encoding='utf-8') as f:
            for item in items:
                f.write(item.strip().decode('utf-8'))
                f.write('\n')

    def IfRegjumpTab():
        try:
            # Attempt to get the current selection and set the insertion row
            selRow = tab1.progView.curselection()[0]
            selRow += 1
        except IndexError:
            selRow = tab1.progView.size() - 1
            tab1.progView.select_set(selRow)
        
        # Get the register number, comparison value, and target tab
        regNum = regNumJmpEntryField.get()
        regEqNum = regEqJmpEntryField.get()
        tabNum = regTabJmpEntryField.get()
        
        # Construct the command string
        tabjmp = f"If Register {regNum} = {regEqNum} Jump to Tab {tabNum}"
        
        # Insert command into progView
        tab1.progView.insert(selRow, bytes(tabjmp + '\n', 'utf-8'))
        tab1.progView.selection_clear(0, ctk.END)
        tab1.progView.select_set(selRow)
        
        # Save all items in progView to file
        items = tab1.progView.get(0, ctk.END)
        file_path = os.path.relpath(ProgEntryField.get())
        
        with open(file_path, 'w', encoding='utf-8') as f:
            for item in items:
                f.write(item.strip().decode('utf-8'))
                f.write('\n')

    def insertRegister():
        try:
            # Attempt to get the current selection and set the insertion row
            selRow = tab1.progView.curselection()[0]
            selRow += 1
        except IndexError:
            selRow = tab1.progView.size() - 1
            tab1.progView.select_set(selRow)
        
        # Get register number and command
        regNum = regNumEntryField.get()
        regCmd = regEqEntryField.get()
        
        # Construct the register command string
        regIns = f"Register {regNum} = {regCmd}"
        
        # Insert command into progView
        tab1.progView.insert(selRow, bytes(regIns + '\n', 'utf-8'))
        tab1.progView.selection_clear(0, ctk.END)
        tab1.progView.select_set(selRow)
        
        # Save all items in progView to file
        items = tab1.progView.get(0, ctk.END)
        file_path = os.path.relpath(ProgEntryField.get())
        
        with open(file_path, 'w', encoding='utf-8') as f:
            for item in items:
                f.write(item.strip().decode('utf-8'))
                f.write('\n')

    def storPos():
        try:
            # Attempt to get the current selection and set the insertion row
            selRow = tab1.progView.curselection()[0]
            selRow += 1
        except IndexError:
            selRow = tab1.progView.size() - 1
            tab1.progView.select_set(selRow)
        
        # Retrieve values from entry fields
        regNum = storPosNumEntryField.get()
        regElmnt = storPosElEntryField.get()
        regCmd = storPosValEntryField.get()
        
        # Construct the position register command string
        regIns = f"Position Register {regNum} Element {regElmnt} = {regCmd}"
        
        # Insert the command into progView
        tab1.progView.insert(selRow, bytes(regIns + '\n', 'utf-8'))
        tab1.progView.selection_clear(0, ctk.END)
        tab1.progView.select_set(selRow)
        
        # Save all items in progView to file
        items = tab1.progView.get(0, ctk.END)
        file_path = os.path.relpath(ProgEntryField.get())
        
        with open(file_path, 'w', encoding='utf-8') as f:
            for item in items:
                f.write(item.strip().decode('utf-8'))
                f.write('\n')

    def insCalibrate():
        try:
            # Attempt to get the current selection and set the insertion row
            selRow = tab1.progView.curselection()[0] + 1
        except IndexError:
            # Default to the end if there is no selection
            selRow = tab1.progView.size() - 1
            tab1.progView.select_set(selRow)
        
        # Define the calibration command
        insCal = "Calibrate Robot"
        
        # Insert the command into progView
        tab1.progView.insert(selRow, bytes(insCal + '\n', 'utf-8'))
        tab1.progView.selection_clear(0, ctk.END)
        tab1.progView.select_set(selRow)
        
        # Save all items in progView to file
        items = tab1.progView.get(0, ctk.END)
        file_path = os.path.relpath(ProgEntryField.get())
        
        with open(file_path, 'w', encoding='utf-8') as f:
            for item in items:
                f.write(item.strip().decode('utf-8'))
                f.write('\n')

    def progViewselect(event):
        try:
            # Get the selected row index in progView
            selRow = tab1.progView.curselection()[0]
            
            # Update curRowEntryField with the selected row index
            curRowEntryField.delete(0, ctk.END)
            curRowEntryField.insert(0, selRow)
        except IndexError:
            # Handle case where no item is selected
            curRowEntryField.delete(0, ctk.END)

    def getSel():
        try:
            # Get the selected row index in progView
            selRow = tab1.progView.curselection()[0]
            
            # Scroll the view to make the selected row visible
            tab1.progView.see(selRow + 2)
            
            data = list(map(int, tab1.progView.curselection()))
            command = tab1.progView.get(data[0]).decode()

            manEntryField.delete(0, ctk.END)
            manEntryField.insert(0, command)
        except IndexError:
            # Handle case where no item is selected
            manEntryField.delete(0, ctk.END)

    def control_servo(servo_number, position_field):
        savePosData()
        servoPos = position_field.get()
        command = f"SV{servo_number}P{servoPos}\n"
        ser2.write(command.encode())
        ser2.flushInput()
        time.sleep(0.1)
        ser2.read()

    # Refactored servo control functions
    def Servo0on():
        control_servo(0, servo0onEntryField)
    def Servo0off():
        control_servo(0, servo0offEntryField)
    def Servo1on():
        control_servo(1, servo1onEntryField)
    def Servo1off():
        control_servo(1, servo1offEntryField)
    def Servo2on():
        control_servo(2, servo2onEntryField)
    def Servo2off():
        control_servo(2, servo2offEntryField)
    def Servo3on():
        control_servo(3, servo3onEntryField)
    def Servo3off():
        control_servo(3, servo3offEntryField)

    def control_output(action, output_field):
        outputNum = output_field.get()
        command = f"{action}X{outputNum}\n"
        ser2.write(command.encode())
        ser2.flushInput()
        time.sleep(0.1)
        ser2.read()

    # Refactored digital output control functions
    def DO1on():
        control_output("ON", DO1onEntryField)
    def DO1off():
        control_output("OF", DO1offEntryField)
    def DO2on():
        control_output("ON", DO2onEntryField)
    def DO2off():
        control_output("OF", DO2offEntryField)
    def DO3on():
        control_output("ON", DO3onEntryField)
    def DO3off():
        control_output("OF", DO3offEntryField)
    def DO4on():
        control_output("ON", DO4onEntryField)
    def DO4off():
        control_output("OF", DO4offEntryField)
    def DO5on():
        control_output("ON", DO5onEntryField)
    def DO5off():
        control_output("OF", DO5offEntryField)
    def DO6on():
        control_output("ON", DO6onEntryField)
    def DO6off():
        control_output("OF", DO6offEntryField)

    def TestString():
        # Construct command and send it
        command = "TM" + testSendEntryField.get() + "\n"
        ser.write(command.encode())
        ser.flushInput()
        
        # Read and display the response
        echo = ser.readline()
        testRecEntryField.delete(0, 'end')
        testRecEntryField.insert(0, echo)

    def ClearTestString():
        # Clear the test receive entry field
        testRecEntryField.delete(0, 'end')

    def CalcLinDist(X2, Y2, Z2):
        global XcurPos, YcurPos, ZcurPos, LineDist

        # Calculate the linear distance between the current position and (X2, Y2, Z2)
        LineDist = (((X2 - XcurPos) ** 2) + ((Y2 - YcurPos) ** 2) + ((Z2 - ZcurPos) ** 2)) ** 0.5
        return LineDist

    def CalcLinVect(X2, Y2, Z2):
        global XcurPos, YcurPos, ZcurPos, Xv, Yv, Zv

        # Calculate the vector components from the current position to (X2, Y2, Z2)
        Xv = X2 - XcurPos
        Yv = Y2 - YcurPos
        Zv = Z2 - ZcurPos

        return Xv, Yv, Zv

    def CalcLinWayPt(CX, CY, CZ, curWayPt):
        global XcurPos, YcurPos, ZcurPos
        
        return CX, CY, CZ

    # Calibration and save defs #

    def calRobotAll():
        def create_calibration_command(stage_values, offsets):
            command = "LL" + "".join(
                f"{chr(65 + i)}{val}" for i, val in enumerate(stage_values + offsets)
            ) + "\n"
            return command

        def send_command(command):
            ser.write(command.encode())
            cmdSentEntryField.delete(0, 'end')
            cmdSentEntryField.insert(0, command)
            ser.flushInput()
            return str(ser.readline().strip(), 'utf-8')

        def handle_response(response, stage):
            success = response.startswith('A')
            displayPosition(response) if success else ErrorHandler(response)
            message = f"Auto Calibration Stage {stage} {'Successful' if success else 'Failed - See Log'}"
            style = "OK.TLabel" if success else "Alarm.TLabel"
            almStatusLab.config(text=message, style=style)
            almStatusLab2.config(text=message, style=style)
            return message

        def update_log(message):
            Curtime = datetime.datetime.now().strftime("%B %d %Y - %I:%M%p")
            tab8.ElogView.insert(END, f"{Curtime} - {message}")
            pickle.dump(tab8.ElogView.get(0, END), open("ErrorLog", "wb"))

        # Stage 1 Calibration
        stage1_values = [
            J1CalStatVal, J2CalStatVal, J3CalStatVal, J4CalStatVal, J5CalStatVal,
            J6CalStatVal, J7CalStatVal, J8CalStatVal, J9CalStatVal
        ]
        offsets = [
            J1calOff, J2calOff, J3calOff, J4calOff, J5calOff, J6calOff, J7calOff,
            J8calOff, J9calOff
        ]
        
        command = create_calibration_command(stage1_values, offsets)
        response = send_command(command)
        message = handle_response(response, stage=1)
        update_log(message)

        # Stage 2 Calibration
        stage2_values = [
            J1CalStatVal2, J2CalStatVal2, J3CalStatVal2, J4CalStatVal2, J5CalStatVal2,
            J6CalStatVal2, J7CalStatVal2, J8CalStatVal2, J9CalStatVal2
        ]
        
        if sum(stage2_values) > 0:
            command = create_calibration_command(stage2_values, offsets)
            response = send_command(command)
            message = handle_response(response, stage=2)
            update_log(message)

    def calibrate_joint(joint_id, joint_command):
        command = f"LL{joint_command}" + "J" + str(J1calOff) + "K" + str(J2calOff) + "L" + str(J3calOff) + "M" + str(
            J4calOff) + "N" + str(J5calOff) + "O" + str(J6calOff) + "P" + str(J7calOff) + "Q" + str(J8calOff) + "R" + str(J9calOff) + "\n"
        ser.write(command.encode())
        cmdSentEntryField.delete(0, 'end')
        cmdSentEntryField.insert(0, command)
        ser.flushInput()
        response = str(ser.readline().strip(), 'utf-8')
        cmdRecEntryField.delete(0, 'end')
        cmdRecEntryField.insert(0, response)
        
        if response.startswith("A"):
            displayPosition(response)
            message = f"J{joint_id} Calibrated Successfully"
            almStatusLab.config(text=message, style="OK.TLabel")
            almStatusLab2.config(text=message, style="OK.TLabel")
        else:
            message = f"J{joint_id} Calibration Failed"
            almStatusLab.config(text=message, style="Alarm.TLabel")
            almStatusLab2.config(text=message, style="Alarm.TLabel")
            ErrorHandler(response)
        
        Curtime = datetime.datetime.now().strftime("%B %d %Y - %I:%M%p")
        tab8.ElogView.insert(END, Curtime + " - " + message)
        value = tab8.ElogView.get(0, END)
        pickle.dump(value, open("ErrorLog", "wb"))

    # Refactored calibration functions for each joint
    def calRobotJ1():
        calibrate_joint(1, "A1B0C0D0E0F0G0H0I0")
    def calRobotJ2():
        calibrate_joint(2, "A0B1C0D0E0F0G0H0I0")
    def calRobotJ3():
        calibrate_joint(3, "A0B0C1D0E0F0G0H0I0")
    def calRobotJ4():
        calibrate_joint(4, "A0B0C0D1E0F0G0H0I0")
    def calRobotJ5():
        calibrate_joint(5, "A0B0C0D0E1F0G0H0I0")
    def calRobotJ6():
        calibrate_joint(6, "A0B0C0D0E0F1G0H0I0")
    def calRobotJ7():
        calibrate_joint(7, "A0B0C0D0E0F0G1H0I0")
    def calRobotJ8():
        calibrate_joint(8, "A0B0C0D0E0F0G0H1I0")
    def calRobotJ9():
        calibrate_joint(9, "A0B0C0D0E0F0G0H0I1")

    def calRobotMid():
        print("foo")
        # add mid command

    def correctPos():
        def send_command(command):
            ser.write(command.encode())
            ser.flushInput()
            time.sleep(0.1)
            return str(ser.readline().strip(), 'utf-8')

        command = "CP\n"
        response = send_command(command)
        displayPosition(response)

    def requestPos():
        def send_command(command):
            ser.write(command.encode())
            ser.flushInput()
            time.sleep(0.1)
            return str(ser.readline().strip(), 'utf-8')

        command = "RP\n"
        response = send_command(command)
        displayPosition(response)

    def updateParams():
        def get_entry_fields():
            params = {
                "TFx": TFxEntryField.get(),
                "TFy": TFyEntryField.get(),
                "TFz": TFzEntryField.get(),
                "TFrz": TFrzEntryField.get(),
                "TFry": TFryEntryField.get(),
                "TFrx": TFrxEntryField.get(),
                "motDir": [J1MotDirEntryField.get(), J2MotDirEntryField.get(), J3MotDirEntryField.get(), J4MotDirEntryField.get(),
                        J5MotDirEntryField.get(), J6MotDirEntryField.get(), J7MotDirEntryField.get(), J8MotDirEntryField.get(), J9MotDirEntryField.get()],
                "calDir": [J1CalDirEntryField.get(), J2CalDirEntryField.get(), J3CalDirEntryField.get(), J4CalDirEntryField.get(),
                        J5CalDirEntryField.get(), J6CalDirEntryField.get(), J7CalDirEntryField.get(), J8CalDirEntryField.get(), J9CalDirEntryField.get()],
                "posLim": [J1PosLimEntryField.get(), J2PosLimEntryField.get(), J3PosLimEntryField.get(), J4PosLimEntryField.get(),
                        J5PosLimEntryField.get(), J6PosLimEntryField.get()],
                "negLim": [J1NegLimEntryField.get(), J2NegLimEntryField.get(), J3NegLimEntryField.get(), J4NegLimEntryField.get(),
                        J5NegLimEntryField.get(), J6NegLimEntryField.get()],
                "stepDeg": [J1StepDegEntryField.get(), J2StepDegEntryField.get(), J3StepDegEntryField.get(),
                            J4StepDegEntryField.get(), J5StepDegEntryField.get(), J6StepDegEntryField.get()],
                "encMult": [str(float(J1EncCPREntryField.get()) / float(J1DriveMSEntryField.get())),
                            str(float(J2EncCPREntryField.get()) / float(J2DriveMSEntryField.get())),
                            str(float(J3EncCPREntryField.get()) / float(J3DriveMSEntryField.get())),
                            str(float(J4EncCPREntryField.get()) / float(J4DriveMSEntryField.get())),
                            str(float(J5EncCPREntryField.get()) / float(J5DriveMSEntryField.get())),
                            str(float(J6EncCPREntryField.get()) / float(J6DriveMSEntryField.get()))],
                "dhTheta": [J1ΘEntryField.get(), J2ΘEntryField.get(), J3ΘEntryField.get(), J4ΘEntryField.get(),
                            J5ΘEntryField.get(), J6ΘEntryField.get()],
                "dhAlpha": [J1αEntryField.get(), J2αEntryField.get(), J3αEntryField.get(), J4αEntryField.get(),
                            J5αEntryField.get(), J6αEntryField.get()],
                "dhDist": [J1dEntryField.get(), J2dEntryField.get(), J3dEntryField.get(), J4dEntryField.get(),
                        J5dEntryField.get(), J6dEntryField.get()],
                "dhLink": [J1aEntryField.get(), J2aEntryField.get(), J3aEntryField.get(), J4aEntryField.get(),
                        J5aEntryField.get(), J6aEntryField.get()]
            }
            return params

        def configure_limits(params):
            limits = zip(
                [J1negLimLab, J2negLimLab, J3negLimLab, J4negLimLab, J5negLimLab, J6negLimLab],
                [J1posLimLab, J2posLimLab, J3posLimLab, J4posLimLab, J5posLimLab, J6posLimLab],
                [J1jogslide, J2jogslide, J3jogslide, J4jogslide, J5jogslide, J6jogslide],
                params["negLim"],
                params["posLim"]
            )
            for negLab, posLab, slide, negLim, posLim in limits:
                negLab.config(text="-" + negLim, style="Jointlim.TLabel")
                posLab.config(text=posLim, style="Jointlim.TLabel")
                slide.config(from_=-float(negLim), to=float(posLim), length=180, orient=HORIZONTAL, command=eval(slide["command"].cget("command")))

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

        ser.write(command.encode())
        ser.flush()
        time.sleep(0.1)
        ser.flushInput()
        time.sleep(0.1)
        response = ser.read_all()

    def calExtAxis():
        def configure_axis(index, pos_limit, neg_limit_label, pos_limit_label, jog_slider, update_command):
            neg_limit = 0  # Constant for all axes in this context
            neg_limit_label.config(text=str(-neg_limit), style="Jointlim.TLabel")
            pos_limit_label.config(text=str(pos_limit), style="Jointlim.TLabel")
            jog_slider.config(from_=-neg_limit, to=pos_limit, length=125, orient=HORIZONTAL, command=update_command)

        # Retrieve and configure limits
        pos_limits = [
            float(axis7lengthEntryField.get()),
            float(axis8lengthEntryField.get()),
            float(axis9lengthEntryField.get())
        ]

        # Configure each axis
        configure_axis(7, pos_limits[0], J7negLimLab, J7posLimLab, J7jogslide, J7sliderUpdate)
        configure_axis(8, pos_limits[1], J8negLimLab, J8posLimLab, J8jogslide, J8sliderUpdate)
        configure_axis(9, pos_limits[2], J9negLimLab, J9posLimLab, J9jogslide, J9sliderUpdate)
        
        # Build command string
        command = (
            f"CEA{pos_limits[0]}B{J7rotation}C{J7steps}"
            f"D{pos_limits[1]}E{J8rotation}F{J8steps}"
            f"G{pos_limits[2]}H{J9rotation}I{J9steps}\n"
        )
        
        # Send command
        ser.write(command.encode())
        ser.flushInput()
        time.sleep(0.1)
        response = ser.read()

    def zero_axis(axis_number, axis_name):
        command = f"Z{axis_number}\n"
        ser.write(command.encode())
        ser.flushInput()
        time.sleep(0.1)
        status_text = f"{axis_name} Calibration Forced to Zero"
        almStatusLab.config(text=status_text, style="Warn.TLabel")
        almStatusLab2.config(text=status_text, style="Warn.TLabel")
        message = f"{axis_name} Calibration Forced to Zero - this is for commissioning and testing - be careful!"
        curtime = datetime.datetime.now().strftime("%B %d %Y - %I:%M%p")
        tab8.ElogView.insert(END, f"{curtime} - {message}")
        value = tab8.ElogView.get(0, END)
        pickle.dump(value, open("ErrorLog", "wb"))
        response = str(ser.readline().strip(), 'utf-8')
        displayPosition(response)

    # Main functions calling the helper function with specific parameters
    def zeroAxis7():
        zero_axis(7, "J7")
    def zeroAxis8():
        zero_axis(8, "J8")
    def zeroAxis9():
        zero_axis(9, "J9")

    def sendPos():
        # Create the command string with formatted current positions
        current_positions = {
            "A": J1AngCur, "B": J2AngCur, "C": J3AngCur, "D": J4AngCur,
            "E": J5AngCur, "F": J6AngCur, "G": J7PosCur, "H": J8PosCur, "I": J9PosCur
        }
        command = "SP" + "".join(f"{key}{value}" for key, value in current_positions.items()) + "\n"
        
        # Send the command
        ser.write(command.encode())
        ser.flushInput()
        time.sleep(0.1)
        response = ser.read()

    def CalZeroPos():
        # Record the current time for logging
        current_time = datetime.datetime.now().strftime("%B %d %Y - %I:%M%p")

        # Send zero calibration command
        command = "SPA0B0C0D0E90F0\n"
        ser.write(command.encode())
        ser.flushInput()
        time.sleep(0.1)
        ser.read()

        # Request updated position and update status labels
        requestPos()
        status_message = "Calibration Forced to Home"
        almStatusLab.config(text=status_message, style="Warn.TLabel")
        almStatusLab2.config(text=status_message, style="Warn.TLabel")

        # Log the calibration event
        log_message = f"{current_time} - {status_message} - this is for commissioning and testing - be careful!"
        tab8.ElogView.insert(END, log_message)
        log_content = tab8.ElogView.get(0, END)
        pickle.dump(log_content, open("ErrorLog", "wb"))

    def CalRestPos():
        # Record the current time for logging
        current_time = datetime.datetime.now().strftime("%B %d %Y - %I:%M%p")

        # Send rest position calibration command
        command = "SPA0B0C-89D0E0F0\n"
        ser.write(command.encode())
        ser.flushInput()
        time.sleep(0.1)
        ser.read()

        # Request updated position and update status labels
        requestPos()
        status_message = "Calibration Forced to Vertical Rest Pos"
        almStatusLab.config(text=status_message, style="Warn.TLabel")
        almStatusLab2.config(text=status_message, style="Warn.TLabel")

        # Log the calibration event
        log_message = f"{current_time} - Calibration Forced to Vertical - this is for commissioning and testing - be careful!"
        tab8.ElogView.insert(END, log_message)
        log_content = tab8.ElogView.get(0, END)
        pickle.dump(log_content, open("ErrorLog", "wb"))

    def displayPosition(response):
        global J1AngCur, J2AngCur, J3AngCur, J4AngCur, J5AngCur, J6AngCur
        global J7StepCur, XcurPos, YcurPos, ZcurPos, RxcurPos, RycurPos, RzcurPos
        global J7PosCur, J8PosCur, J9PosCur, WC

        # Update received command in entry field
        cmdRecEntryField.delete(0, 'end')
        cmdRecEntryField.insert(0, response)

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

        # Assign parsed data to globals
        for key, value in parsed_data.items():
            globals()[key] = value

        # Determine wrist configuration
        WC = "F" if float(parsed_data["J5AngCur"]) > 0 else "N"

        # Update GUI elements
        entry_fields = [
            (J1curAngEntryField, parsed_data["J1AngCur"]),
            (J2curAngEntryField, parsed_data["J2AngCur"]),
            (J3curAngEntryField, parsed_data["J3AngCur"]),
            (J4curAngEntryField, parsed_data["J4AngCur"]),
            (J5curAngEntryField, parsed_data["J5AngCur"]),
            (J6curAngEntryField, parsed_data["J6AngCur"]),
            (XcurEntryField, parsed_data["XcurPos"]),
            (YcurEntryField, parsed_data["YcurPos"]),
            (ZcurEntryField, parsed_data["ZcurPos"]),
            (RzcurEntryField, parsed_data["RzcurPos"]),
            (RycurEntryField, parsed_data["RycurPos"]),
            (RxcurEntryField, parsed_data["RxcurPos"]),
            (J7curAngEntryField, parsed_data["J7PosCur"]),
            (J8curAngEntryField, parsed_data["J8PosCur"]),
            (J9curAngEntryField, parsed_data["J9PosCur"]),
            (manEntryField, parsed_data["Debug"])
        ]
        for field, value in entry_fields:
            field.delete(0, 'end')
            field.insert(0, value)

        # Update sliders
        jog_sliders = [
            (J1jogslide, parsed_data["J1AngCur"]),
            (J2jogslide, parsed_data["J2AngCur"]),
            (J3jogslide, parsed_data["J3AngCur"]),
            (J4jogslide, parsed_data["J4AngCur"]),
            (J5jogslide, parsed_data["J5AngCur"]),
            (J6jogslide, parsed_data["J6AngCur"]),
            (J7jogslide, parsed_data["J7PosCur"]),
            (J8jogslide, parsed_data["J8PosCur"]),
            (J9jogslide, parsed_data["J9PosCur"])
        ]
        for slider, value in jog_sliders:
            slider.set(value)

        # Save position data and handle errors
        savePosData()
        if parsed_data["Flag"]:
            ErrorHandler(parsed_data["Flag"])
        if parsed_data["SpeedVioation"] == '1':
            current_time = datetime.datetime.now().strftime("%B %d %Y - %I:%M%p")
            message = "Max Speed Violation - Reduce Speed Setpoint or Travel Distance"
            tab8.ElogView.insert(END, f"{current_time} - {message}")
            pickle.dump(tab8.ElogView.get(0, END), open("ErrorLog", "wb"))
            almStatusLab.config(text=message, style="Warn.TLabel")
            almStatusLab2.config(text=message, style="Warn.TLabel")

    def ClearKinTabFields():
        # Define field groups for organized clearing
        motion_dir_fields = [
            J1MotDirEntryField, J2MotDirEntryField, J3MotDirEntryField, J4MotDirEntryField,
            J5MotDirEntryField, J6MotDirEntryField, J7MotDirEntryField, J8MotDirEntryField, J9MotDirEntryField
        ]
        
        calibration_dir_fields = [
            J1CalDirEntryField, J2CalDirEntryField, J3CalDirEntryField, J4CalDirEntryField,
            J5CalDirEntryField, J6CalDirEntryField, J7CalDirEntryField, J8CalDirEntryField, J9CalDirEntryField
        ]
        
        position_limit_fields = [
            J1PosLimEntryField, J1NegLimEntryField, J2PosLimEntryField, J2NegLimEntryField,
            J3PosLimEntryField, J3NegLimEntryField, J4PosLimEntryField, J4NegLimEntryField,
            J5PosLimEntryField, J5NegLimEntryField, J6PosLimEntryField, J6NegLimEntryField
        ]
        
        step_deg_fields = [
            J1StepDegEntryField, J2StepDegEntryField, J3StepDegEntryField,
            J4StepDegEntryField, J5StepDegEntryField, J6StepDegEntryField
        ]
        
        drive_ms_fields = [
            J1DriveMSEntryField, J2DriveMSEntryField, J3DriveMSEntryField,
            J4DriveMSEntryField, J5DriveMSEntryField, J6DriveMSEntryField
        ]
        
        encoder_cpr_fields = [
            J1EncCPREntryField, J2EncCPREntryField, J3EncCPREntryField,
            J4EncCPREntryField, J5EncCPREntryField, J6EncCPREntryField
        ]
        
        theta_fields = [
            J1ΘEntryField, J2ΘEntryField, J3ΘEntryField, J4ΘEntryField,
            J5ΘEntryField, J6ΘEntryField
        ]
        
        alpha_fields = [
            J1αEntryField, J2αEntryField, J3αEntryField, J4αEntryField,
            J5αEntryField, J6αEntryField
        ]
        
        d_fields = [
            J1dEntryField, J2dEntryField, J3dEntryField, J4dEntryField,
            J5dEntryField, J6dEntryField
        ]
        
        a_fields = [
            J1aEntryField, J2aEntryField, J3aEntryField, J4aEntryField,
            J5aEntryField, J6aEntryField
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

    def LoadAR4Mk3default():
        ClearKinTabFields()

        # Define default values for each entry field
        default_values = {
            # Motor directions
            J1MotDirEntryField: 0, J2MotDirEntryField: 1, J3MotDirEntryField: 1,
            J4MotDirEntryField: 1, J5MotDirEntryField: 1, J6MotDirEntryField: 1,
            J7MotDirEntryField: 1, J8MotDirEntryField: 1, J9MotDirEntryField: 1,
            
            # Calibration directions
            J1CalDirEntryField: 1, J2CalDirEntryField: 0, J3CalDirEntryField: 1,
            J4CalDirEntryField: 0, J5CalDirEntryField: 0, J6CalDirEntryField: 1,
            J7CalDirEntryField: 0, J8CalDirEntryField: 0, J9CalDirEntryField: 0,
            
            # Position limits
            J1PosLimEntryField: 170, J1NegLimEntryField: 170, J2PosLimEntryField: 90,
            J2NegLimEntryField: 42, J3PosLimEntryField: 52, J3NegLimEntryField: 89,
            J4PosLimEntryField: 180, J4NegLimEntryField: 180, J5PosLimEntryField: 105,
            J5NegLimEntryField: 105, J6PosLimEntryField: 180, J6NegLimEntryField: 180,
            
            # Steps per degree
            J1StepDegEntryField: 44.4444, J2StepDegEntryField: 55.5555,
            J3StepDegEntryField: 55.5555, J4StepDegEntryField: 49.7777,
            J5StepDegEntryField: 21.8602, J6StepDegEntryField: 22.2222,
            
            # Drive MS settings
            J1DriveMSEntryField: 400, J2DriveMSEntryField: 400, J3DriveMSEntryField: 400,
            J4DriveMSEntryField: 400, J5DriveMSEntryField: 800, J6DriveMSEntryField: 400,
            
            # Encoder CPR settings
            J1EncCPREntryField: 4000, J2EncCPREntryField: 4000, J3EncCPREntryField: 4000,
            J4EncCPREntryField: 4000, J5EncCPREntryField: 4000, J6EncCPREntryField: 4000,
            
            # Θ (Theta) angles
            J1ΘEntryField: 0, J2ΘEntryField: -90, J3ΘEntryField: 0,
            J4ΘEntryField: 0, J5ΘEntryField: 0, J6ΘEntryField: 180,
            
            # α (Alpha) angles
            J1αEntryField: 0, J2αEntryField: -90, J3αEntryField: 0,
            J4αEntryField: -90, J5αEntryField: 90, J6αEntryField: -90,
            
            # d distances
            J1dEntryField: 169.77, J2dEntryField: 0, J3dEntryField: 0,
            J4dEntryField: 222.63, J5dEntryField: 0, J6dEntryField: 41,
            
            # a distances
            J1aEntryField: 0, J2aEntryField: 64.2, J3aEntryField: 305,
            J4aEntryField: 0, J5aEntryField: 0, J6aEntryField: 0
        }

        # Insert default values into each entry field
        for entry_field, value in default_values.items():
            entry_field.insert(0, str(value))

    ## Profiles defs ##

    def SaveAndApplyCalibration():
        global J1AngCur, J2AngCur, J3AngCur, J4AngCur, J5AngCur, J6AngCur
        global XcurPos, YcurPos, ZcurPos, RxcurPos, RycurPos, RzcurPos
        global J7PosCur, J8PosCur, J9PosCur, VisFileLoc, VisProg
        global VisOrigXpix, VisOrigXmm, VisOrigYpix, VisOrigYmm
        global VisEndXpix, VisEndXmm, VisEndYpix, VisEndYmm
        global J1calOff, J2calOff, J3calOff, J4calOff, J5calOff, J6calOff, J7calOff, J8calOff, J9calOff
        global J1OpenLoopVal, J2OpenLoopVal, J3OpenLoopVal, J4OpenLoopVal, J5OpenLoopVal, J6OpenLoopVal
        global DisableWristRotVal
        global J1CalStatVal, J2CalStatVal, J3CalStatVal, J4CalStatVal, J5CalStatVal, J6CalStatVal
        global J7CalStatVal, J8CalStatVal, J9CalStatVal
        global J1CalStatVal2, J2CalStatVal2, J3CalStatVal2, J4CalStatVal2, J5CalStatVal2, J6CalStatVal2
        global J7CalStatVal2, J8CalStatVal2, J9CalStatVal2
        global J7PosLim, J7rotation, J7steps, J8length, J8rotation, J8steps, J9length, J9rotation, J9steps
        global IncJogStat

        # Set values from GUI inputs
        J7PosCur = J7curAngEntryField.get()
        J8PosCur = J8curAngEntryField.get()
        J9PosCur = J9curAngEntryField.get()
        VisProg = visoptions.get()
        J1calOff = float(J1calOffEntryField.get())
        J2calOff = float(J2calOffEntryField.get())
        J3calOff = float(J3calOffEntryField.get())
        J4calOff = float(J4calOffEntryField.get())
        J5calOff = float(J5calOffEntryField.get())
        J6calOff = float(J6calOffEntryField.get())
        J7calOff = float(J7calOffEntryField.get())
        J8calOff = float(J8calOffEntryField.get())
        J9calOff = float(J9calOffEntryField.get())
        J1OpenLoopVal = int(J1OpenLoopStat.get())
        J2OpenLoopVal = int(J2OpenLoopStat.get())
        J3OpenLoopVal = int(J3OpenLoopStat.get())
        J4OpenLoopVal = int(J4OpenLoopStat.get())
        J5OpenLoopVal = int(J5OpenLoopStat.get())
        J6OpenLoopVal = int(J6OpenLoopStat.get())
        DisableWristRotVal = int(DisableWristRot.get())
        J1CalStatVal = int(J1CalStat.get())
        J2CalStatVal = int(J2CalStat.get())
        J3CalStatVal = int(J3CalStat.get())
        J4CalStatVal = int(J4CalStat.get())
        J5CalStatVal = int(J5CalStat.get())
        J6CalStatVal = int(J6CalStat.get())
        J7CalStatVal = int(J7CalStat.get())
        J8CalStatVal = int(J8CalStat.get())
        J9CalStatVal = int(J9CalStat.get())
        J1CalStatVal2 = int(J1CalStat2.get())
        J2CalStatVal2 = int(J2CalStat2.get())
        J3CalStatVal2 = int(J3CalStat2.get())
        J4CalStatVal2 = int(J4CalStat2.get())
        J5CalStatVal2 = int(J5CalStat2.get())
        J6CalStatVal2 = int(J6CalStat2.get())
        J7CalStatVal2 = int(J7CalStat2.get())
        J8CalStatVal2 = int(J8CalStat2.get())
        J9CalStatVal2 = int(J9CalStat2.get())
        J7PosLim = float(axis7lengthEntryField.get())
        J7rotation = float(axis7rotEntryField.get())
        J7steps = float(axis7stepsEntryField.get())
        J8length = float(axis8lengthEntryField.get())
        J8rotation = float(axis8rotEntryField.get())
        J8steps = float(axis8stepsEntryField.get())
        J9length = float(axis9lengthEntryField.get())
        J9rotation = float(axis9rotEntryField.get())
        J9steps = float(axis9stepsEntryField.get())

        # Apply the updates and save
        try:
            updateParams()
            time.sleep(0.1)
            calExtAxis()
        except:
            print("No serial connection with Teensy board")

        savePosData()

    def savePosData():
        global J1AngCur, J2AngCur, J3AngCur, J4AngCur, J5AngCur, J6AngCur
        global XcurPos, YcurPos, ZcurPos, RxcurPos, RycurPos, RzcurPos, curTheme
        global J7PosLim, J7rotation, J7steps
        global J8length, J8rotation, J8steps
        global J9length, J9rotation, J9steps
        global mX1, mY1, mX2, mY2

        # Clear the calibration list and insert values sequentially
        calibration.delete(0, END)
        
        # Joint Angles
        calibration.insert(END, J1AngCur)
        calibration.insert(END, J2AngCur)
        calibration.insert(END, J3AngCur)
        calibration.insert(END, J4AngCur)
        calibration.insert(END, J5AngCur)
        calibration.insert(END, J6AngCur)

        # Current Positions (X, Y, Z, Rz, Ry, Rx)
        calibration.insert(END, XcurPos)
        calibration.insert(END, YcurPos)
        calibration.insert(END, ZcurPos)
        calibration.insert(END, RzcurPos)
        calibration.insert(END, RycurPos)
        calibration.insert(END, RxcurPos)

        # Ports and Program Entry Fields
        calibration.insert(END, comPortEntryField.get())
        calibration.insert(END, ProgEntryField.get())
        calibration.insert(END, servo0onEntryField.get())
        calibration.insert(END, servo0offEntryField.get())
        calibration.insert(END, servo1onEntryField.get())
        calibration.insert(END, servo1offEntryField.get())
        calibration.insert(END, DO1onEntryField.get())
        calibration.insert(END, DO1offEntryField.get())
        calibration.insert(END, DO2onEntryField.get())
        calibration.insert(END, DO2offEntryField.get())

        # Transform Fields (TFx to TFrz)
        calibration.insert(END, TFxEntryField.get())
        calibration.insert(END, TFyEntryField.get())
        calibration.insert(END, TFzEntryField.get())
        calibration.insert(END, TFrxEntryField.get())
        calibration.insert(END, TFryEntryField.get())
        calibration.insert(END, TFrzEntryField.get())

        # Joint 7 to 9 Calibration Fields
        calibration.insert(END, J7curAngEntryField.get())
        calibration.insert(END, J8curAngEntryField.get())
        calibration.insert(END, J9curAngEntryField.get())

        # Visual Calibration Fields
        calibration.insert(END, "VisFileLocEntryField")  # Placeholder
        calibration.insert(END, visoptions.get())
        calibration.insert(END, "VisPicOxPEntryField")
        calibration.insert(END, "VisPicOxMEntryField")
        calibration.insert(END, "VisPicOyPEntryField")
        calibration.insert(END, "VisPicOyMEntryField")
        calibration.insert(END, "VisPicXPEntryField")
        calibration.insert(END, "VisPicXMEntryField")
        calibration.insert(END, "VisPicYPEntryField")
        calibration.insert(END, "VisPicYMEntryField")

        # Calibration Offsets (J1 to J6)
        calibration.insert(END, J1calOffEntryField.get())
        calibration.insert(END, J2calOffEntryField.get())
        calibration.insert(END, J3calOffEntryField.get())
        calibration.insert(END, J4calOffEntryField.get())
        calibration.insert(END, J5calOffEntryField.get())
        calibration.insert(END, J6calOffEntryField.get())

        # Open Loop Values (J1 to J6)
        calibration.insert(END, J1OpenLoopVal)
        calibration.insert(END, J2OpenLoopVal)
        calibration.insert(END, J3OpenLoopVal)
        calibration.insert(END, J4OpenLoopVal)
        calibration.insert(END, J5OpenLoopVal)
        calibration.insert(END, J6OpenLoopVal)

        # Additional Configuration Fields
        calibration.insert(END, com2PortEntryField.get())
        calibration.insert(END, curTheme)
        calibration.insert(END, J1CalStatVal)
        calibration.insert(END, J2CalStatVal)
        calibration.insert(END, J3CalStatVal)
        calibration.insert(END, J4CalStatVal)
        calibration.insert(END, J5CalStatVal)
        calibration.insert(END, J6CalStatVal)

        # Joint 7 Calibration Parameters
        calibration.insert(END, J7PosLim)
        calibration.insert(END, J7rotation)
        calibration.insert(END, J7steps)
        calibration.insert(END, J7StepCur)

        # Joint Calibration Status Values (2nd Set)
        calibration.insert(END, J1CalStatVal2)
        calibration.insert(END, J2CalStatVal2)
        calibration.insert(END, J3CalStatVal2)
        calibration.insert(END, J4CalStatVal2)
        calibration.insert(END, J5CalStatVal2)
        calibration.insert(END, J6CalStatVal2)

        # Visual Settings
        calibration.insert(END, VisBrightSlide.get())
        calibration.insert(END, VisContrastSlide.get())
        calibration.insert(END, VisBacColorEntryField.get())
        calibration.insert(END, VisScoreEntryField.get())
        calibration.insert(END, VisX1PixEntryField.get())
        calibration.insert(END, VisY1PixEntryField.get())
        calibration.insert(END, VisX2PixEntryField.get())
        calibration.insert(END, VisY2PixEntryField.get())
        calibration.insert(END, VisX1RobEntryField.get())
        calibration.insert(END, VisY1RobEntryField.get())
        calibration.insert(END, VisX2RobEntryField.get())
        calibration.insert(END, VisY2RobEntryField.get())
        calibration.insert(END, VisZoomSlide.get())

        # Other Options
        calibration.insert(END, pick180.get())
        calibration.insert(END, pickClosest.get())
        calibration.insert(END, visoptions.get())
        calibration.insert(END, fullRot.get())
        calibration.insert(END, autoBG.get())

        # Miscellaneous Parameters
        calibration.insert(END, mX1)
        calibration.insert(END, mY1)
        calibration.insert(END, mX2)
        calibration.insert(END, mY2)

        # Joint 8 and 9 Parameters
        calibration.insert(END, J8length)
        calibration.insert(END, J8rotation)
        calibration.insert(END, J8steps)
        calibration.insert(END, J9length)
        calibration.insert(END, J9rotation)
        calibration.insert(END, J9steps)

        # Joint Calibration Offsets (J7 to J9)
        calibration.insert(END, J7calOffEntryField.get())
        calibration.insert(END, J8calOffEntryField.get())
        calibration.insert(END, J9calOffEntryField.get())

        # General Calibration Settings (GC_ST)
        calibration.insert(END, GC_ST_E1_EntryField.get())
        calibration.insert(END, GC_ST_E2_EntryField.get())
        calibration.insert(END, GC_ST_E3_EntryField.get())
        calibration.insert(END, GC_ST_E4_EntryField.get())
        calibration.insert(END, GC_ST_E5_EntryField.get())
        calibration.insert(END, GC_ST_E6_EntryField.get())
        calibration.insert(END, GC_SToff_E1_EntryField.get())
        calibration.insert(END, GC_SToff_E2_EntryField.get())
        calibration.insert(END, GC_SToff_E3_EntryField.get())
        calibration.insert(END, GC_SToff_E4_EntryField.get())
        calibration.insert(END, GC_SToff_E5_EntryField.get())
        calibration.insert(END, GC_SToff_E6_EntryField.get())

        # Wrist Rotation Disable
        calibration.insert(END, DisableWristRotVal)

        # Motor Direction Fields (J1 to J9)
        calibration.insert(END, J1MotDirEntryField.get())
        calibration.insert(END, J2MotDirEntryField.get())
        calibration.insert(END, J3MotDirEntryField.get())
        calibration.insert(END, J4MotDirEntryField.get())
        calibration.insert(END, J5MotDirEntryField.get())
        calibration.insert(END, J6MotDirEntryField.get())
        calibration.insert(END, J7MotDirEntryField.get())
        calibration.insert(END, J8MotDirEntryField.get())
        calibration.insert(END, J9MotDirEntryField.get())

        # Calibration Direction Fields (J1 to J9)
        calibration.insert(END, J1CalDirEntryField.get())
        calibration.insert(END, J2CalDirEntryField.get())
        calibration.insert(END, J3CalDirEntryField.get())
        calibration.insert(END, J4CalDirEntryField.get())
        calibration.insert(END, J5CalDirEntryField.get())
        calibration.insert(END, J6CalDirEntryField.get())
        calibration.insert(END, J7CalDirEntryField.get())
        calibration.insert(END, J8CalDirEntryField.get())
        calibration.insert(END, J9CalDirEntryField.get())

        # Position Limits Fields (J1 to J9)
        calibration.insert(END, J1PosLimEntryField.get())
        calibration.insert(END, J1NegLimEntryField.get())
        calibration.insert(END, J2PosLimEntryField.get())
        calibration.insert(END, J2NegLimEntryField.get())
        calibration.insert(END, J3PosLimEntryField.get())
        calibration.insert(END, J3NegLimEntryField.get())
        calibration.insert(END, J4PosLimEntryField.get())
        calibration.insert(END, J4NegLimEntryField.get())
        calibration.insert(END, J5PosLimEntryField.get())
        calibration.insert(END, J5NegLimEntryField.get())
        calibration.insert(END, J6PosLimEntryField.get())
        calibration.insert(END, J6NegLimEntryField.get())
        calibration.insert(END, J7PosLimEntryField.get())
        calibration.insert(END, J7NegLimEntryField.get())
        calibration.insert(END, J8PosLimEntryField.get())
        calibration.insert(END, J8NegLimEntryField.get())
        calibration.insert(END, J9PosLimEntryField.get())
        calibration.insert(END, J9NegLimEntryField.get())

        # Encoder Settings (J1 to J9)
        calibration.insert(END, J1EncCPREntryField.get())
        calibration.insert(END, J2EncCPREntryField.get())
        calibration.insert(END, J3EncCPREntryField.get())
        calibration.insert(END, J4EncCPREntryField.get())
        calibration.insert(END, J5EncCPREntryField.get())
        calibration.insert(END, J6EncCPREntryField.get())

        # Drive Modes (J1 to J6)
        calibration.insert(END, J1DriveMSEntryField.get())
        calibration.insert(END, J2DriveMSEntryField.get())
        calibration.insert(END, J3DriveMSEntryField.get())
        calibration.insert(END, J4DriveMSEntryField.get())
        calibration.insert(END, J5DriveMSEntryField.get())
        calibration.insert(END, J6DriveMSEntryField.get())

        # Step Degrees (J1 to J6)
        calibration.insert(END, J1StepDegEntryField.get())
        calibration.insert(END, J2StepDegEntryField.get())
        calibration.insert(END, J3StepDegEntryField.get())
        calibration.insert(END, J4StepDegEntryField.get())
        calibration.insert(END, J5StepDegEntryField.get())
        calibration.insert(END, J6StepDegEntryField.get())
        
        # Serialize and save the data
        value = calibration.get(0, END)
        pickle.dump(value, open("ARbot.cal", "wb"))

    def checkSpeedVals():
        speedtype = speedOption.get()
        
        # Validate and update a field with a default value if out of bounds.
        def validate_and_update(field, value, min_val, max_val=None):
            speed = float(field.get())
            if speed <= min_val or (max_val is not None and speed > max_val):
                field.delete(0, 'end')
                field.insert(0, str(value))
        
        # Validate speed based on type
        if speedtype == "mm per Sec":
            validate_and_update(speedEntryField, 5, 0.01)
        elif speedtype == "Seconds":
            validate_and_update(speedEntryField, 1, 0.001)
        elif speedtype == "Percent":
            validate_and_update(speedEntryField, 10, 0.01, 100)
        
        # Validate acceleration, deceleration, and ramp fields
        validate_and_update(ACCspeedField, 10, 0.01, 100)
        validate_and_update(DECspeedField, 10, 0.01, 100)
        validate_and_update(ACCrampField, 50, 0.01, 100)

    def ErrorHandler(response):
        global estopActive, posOutreach
        Curtime = datetime.datetime.now().strftime("%B %d %Y - %I:%M%p")
        
        # Log error message to ElogView and save to a file.
        def log_error(message):
            tab8.ElogView.insert(END, f"{Curtime} - {message}")
            pickle.dump(tab8.ElogView.get(0, END), open("ErrorLog", "wb"))

        # Update alarm labels with a specific message and style.
        def update_alarm_status(message):
            almStatusLab.config(text=message, style="Alarm.TLabel")
            almStatusLab2.config(text=message, style="Alarm.TLabel")
            GCalmStatusLab.config(text=message, style="Alarm.TLabel")

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
                    correctPos()
                    stopProg()
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

        cmdRecEntryField.delete(0, 'end')
        cmdRecEntryField.insert(0, response)
        
        # Axis Limit Error
        if response[1:2] == 'L':
            handle_axis_limit_error()
            stopProg()

        # Collision Error
        elif response[1:2] == 'C':
            handle_collision_error()

        # Position Out of Reach
        elif response[1:2] == 'R':
            posOutreach = True
            stopProg()
            log_error("Position Out of Reach")
            update_alarm_status("Position Out of Reach")

        # Spline Error
        elif response[1:2] == 'S':
            stopProg()
            log_error("Spline Can Only Have Move L Types")
            update_alarm_status("Spline Can Only Have Move L Types")

        # GCode Error
        elif response[1:2] == 'G':
            stopProg()
            log_error("Gcode file not found")
            update_alarm_status("Gcode file not found")

        # Estop Button Pressed
        elif response[1:2] == 'B':
            estopActive = True
            stopProg()
            log_error("Estop Button was Pressed")
            update_alarm_status("Estop Button was Pressed")

        # Calibration Error
        elif response[1:2] == 'A':
            handle_calibration_error()

        # Unknown Error
        else:
            stopProg()
            log_error("Unknown Error")
            update_alarm_status("Unknown Error")

    # Vision defs #

    def testvis():
        visprog = visoptions.get()
        visprog_functions = {
            "Openvision": openvision,
            "Roborealm 1.7.5": roborealm175,
            "x,y,r": xyr
        }
        # Call the function if the visprog option exists in the mapping
        if visprog in visprog_functions:
            visprog_functions[visprog]()

    def openvision():
        global Xpos, Ypos, VisEndYmm
        visfail = 1

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
        while visfail:
            update_status(almStatusLab)
            update_status(almStatusLab2)
            value = read_last_line(VisFileLoc)

            x = int(value[110:122])
            y = int(value[130:142])
            viscalc(x, y)

            visfail = Ypos > VisEndYmm
            if visfail:
                time.sleep(0.1)

        open(VisFileLoc, "w").close()  # Clear the vision file

        # Update fields with position data
        update_entry(VisXfindEntryField, Xpos)
        update_entry(VisYfindEntryField, Ypos)
        update_entry(VisRZfindEntryField, 0)
        update_entry(VisXpixfindEntryField, x)
        update_entry(VisYpixfindEntryField, y)
        update_entry(SP_1_E1_EntryField, Xpos)
        update_entry(SP_1_E2_EntryField, Ypos)

    def roborealm175():
        global Xpos, Ypos, VisEndYmm
        visfail = 1

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
        while visfail:
            update_status(almStatusLab, "WAITING FOR CAMERA", "Alarm.TLabel")
            update_status(almStatusLab2, "WAITING FOR CAMERA", "Alarm.TLabel")

            value = read_last_line(VisFileLoc)

            update_status(almStatusLab, "SYSTEM READY", "OK.TLabel")
            update_status(almStatusLab2, "SYSTEM READY", "OK.TLabel")

            # Extract x and y values from the comma-separated value
            index = value.index(',')
            x = float(value[:index])
            y = float(value[index+1:])
            
            viscalc(x, y)

            visfail = float(Ypos) > float(VisEndYmm)
            if visfail:
                time.sleep(0.1)

        open(VisFileLoc, "w").close()  # Clear the vision file

        # Update fields with position data
        update_entry(VisXfindEntryField, Xpos)
        update_entry(VisYfindEntryField, Ypos)
        update_entry(VisRZfindEntryField, 0)
        update_entry(VisXpixfindEntryField, x)
        update_entry(VisYpixfindEntryField, y)
        update_entry(SP_1_E1_EntryField, Xpos)
        update_entry(SP_1_E2_EntryField, Ypos)

    def xyr():
        global Xpos, Ypos, VisEndYmm
        visfail = 1

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
        while visfail:
            update_status(almStatusLab, "SYSTEM READY", "OK.TLabel")
            update_status(almStatusLab2, "SYSTEM READY", "OK.TLabel")

            value = read_last_line(VisFileLoc)

            # Update status to indicate system is ready
            update_status(almStatusLab, "SYSTEM READY", "OK.TLabel")
            update_status(almStatusLab2, "SYSTEM READY", "OK.TLabel")

            # Parse x, y, r values from comma-separated string
            index1 = value.index(',')
            x = float(value[:index1])

            remaining_value = value[index1 + 1:]
            index2 = remaining_value.index(',')
            y = float(remaining_value[:index2])
            r = float(remaining_value[index2 + 1:])

            viscalc(x, y)

            visfail = Ypos > float(VisEndYmm)
            if visfail:
                time.sleep(0.1)

        open(VisFileLoc, "w").close()  # Clear the vision file

        # Update fields with position and rotation data
        update_entry(VisXfindEntryField, Xpos)
        update_entry(VisYfindEntryField, Ypos)
        update_entry(VisRZfindEntryField, r)
        update_entry(VisXpixfindEntryField, x)
        update_entry(VisYpixfindEntryField, y)
        update_entry(SP_1_E1_EntryField, Xpos)
        update_entry(SP_1_E2_EntryField, Ypos)
        update_entry(SP_1_E3_EntryField, r)

    def viscalc():
        global xMMpos, yMMpos

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
        VisOrigXpix = get_entry_float(VisX1PixEntryField)
        VisOrigXmm = get_entry_float(VisX1RobEntryField)
        VisOrigYpix = get_entry_float(VisY1PixEntryField)
        VisOrigYmm = get_entry_float(VisY1RobEntryField)

        VisEndXpix = get_entry_float(VisX2PixEntryField)
        VisEndXmm = get_entry_float(VisX2RobEntryField)
        VisEndYpix = get_entry_float(VisY2PixEntryField)
        VisEndYmm = get_entry_float(VisY2RobEntryField)

        # Target pixel coordinates to be converted
        x = get_entry_float(VisRetXpixEntryField)
        y = get_entry_float(VisRetYpixEntryField)

        # Calculate mm positions for x and y based on pixel inputs
        xMMpos = calculate_mm_position(VisOrigXpix, VisEndXpix, VisOrigXmm, VisEndXmm, x)
        yMMpos = calculate_mm_position(VisOrigYpix, VisEndYpix, VisOrigYmm, VisEndYmm, y)

        return xMMpos, yMMpos

    ## Define function to show frame ##

    def show_frame():
        if cam_on:
            ret, frame = cap.read()

            if ret:
                # Convert the frame to RGB and resize for display
                cv2image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                img = Image.fromarray(cv2image).resize((480, 320))
                imgtk = ImageTk.PhotoImage(image=img)
                
                # Update customtkinter label with the new frame
                live_lbl.imgtk = imgtk
                live_lbl.configure(image=imgtk)

            # Schedule the next frame update
            live_lbl.after(10, show_frame)

    def start_vid():
        global cam_on, cap
        stop_vid()  # Ensure any previous video capture is stopped
        cam_on = True

        # Get the selected camera index
        selectedCam = camList.index(visoptions.get()) if visoptions.get() in camList else 0
        cap = cv2.VideoCapture(selectedCam)  # Open the selected camera

        show_frame()

    def stop_vid():
        global cam_on, cap
        cam_on = False

        if cap and cap.isOpened():
            cap.release()

    ## Define function to show frame ##

    def take_pic():
        global selectedCam, cap, BGavg, mX1, mY1, mX2, mY2

        # Capture frame from selected camera
        if cam_on:
            ret, frame = cap.read()
        else:
            selectedCam = camList.index(visoptions.get())
            cap = cv2.VideoCapture(selectedCam)
            ret, frame = cap.read()

        # Apply brightness and contrast adjustments
        brightness = int(VisBrightSlide.get())
        contrast = int(VisContrastSlide.get())
        zoom = int(VisZoomSlide.get())

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
        if autoBG.get():
            bg_points = [
                cv2image[int(VisX1PixEntryField.get()), int(VisY1PixEntryField.get())],
                cv2image[int(VisX1PixEntryField.get()), int(VisY2PixEntryField.get())],
                cv2image[int(VisX2PixEntryField.get()), int(VisY2PixEntryField.get())]
            ]
            BGavg = (avg := int(np.mean(bg_points)), avg, avg)
            background = avg
            VisBacColorEntryField.configure(state='enabled')
            VisBacColorEntryField.delete(0, 'end')
            VisBacColorEntryField.insert(0, str(BGavg))
            VisBacColorEntryField.configure(state='disabled')
        else:
            temp = VisBacColorEntryField.get()
            background = int(temp[temp.find("(") + 1 : temp.find(",")])

        # Apply background mask to image
        mask = np.ones_like(cv2image) * background
        mask[mY1:mY2, mX1:mX2] = cv2image[mY1:mY2, mX1:mX2]
        cv2image = mask

        # Update UI with processed image
        img = Image.fromarray(cv2image).resize((640, 480))
        imgtk = ImageTk.PhotoImage(image=img)
        vid_lbl.imgtk = imgtk
        vid_lbl.configure(image=imgtk)

        # Save the image
        cv2.imwrite('curImage.jpg', cv2image)

    def mask_pic():
        global selectedCam, cap, BGavg, mX1, mY1, mX2, mY2

        # Capture frame from selected camera
        if cam_on:
            ret, frame = cap.read()
        else:
            selectedCam = camList.index(visoptions.get())
            cap = cv2.VideoCapture(selectedCam)
            ret, frame = cap.read()

        # Apply brightness and contrast adjustments
        brightness = int(VisBrightSlide.get())
        contrast = int(VisContrastSlide.get())
        zoom = int(VisZoomSlide.get())

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

    def mask_crop(event, x, y, flags, param):
        global x_start, y_start, x_end, y_end, cropping, button_down
        global mX1, mY1, mX2, mY2, background, cropDone

        cropDone = False

        # Set a background color based on autoBG toggle.
        def handle_bg_color():
            autoBGVal = int(autoBG.get())
            if autoBGVal == 1:
                BG1 = oriImage[int(VisX1PixEntryField.get())][int(VisY1PixEntryField.get())]
                BG2 = oriImage[int(VisX1PixEntryField.get())][int(VisY2PixEntryField.get())]
                BG3 = oriImage[int(VisX2PixEntryField.get())][int(VisY2PixEntryField.get())]
                avg = int(mean([BG1, BG2, BG3]))
                BGavg = (avg, avg, avg)
                VisBacColorEntryField.configure(state='enabled')
                VisBacColorEntryField.delete(0, 'end')
                VisBacColorEntryField.insert(0, str(BGavg))
                VisBacColorEntryField.configure(state='disabled')
                return avg
            else:
                return eval(VisBacColorEntryField.get())

        # Apply the selected background to areas outside the crop region.
        def crop_image_with_bg():
            h, w = oriImage.shape[:2]
            for y in range(h):
                for x in range(w):
                    if x >= mX2 or x <= mX1 or y <= mY1 or y >= mY2:
                        oriImage[y, x] = background

        # Update and display the image on the UI.
        def update_displayed_image():
            img = Image.fromarray(oriImage)
            imgtk = ImageTk.PhotoImage(image=img)
            vid_lbl.imgtk = imgtk
            vid_lbl.configure(image=imgtk)
            filename = 'curImage.jpg'
            cv2.imwrite(filename, oriImage)
            cv2.destroyAllWindows()

        # Mouse button down event
        if not button_down and event == cv2.EVENT_LBUTTONDOWN:
            x_start, y_start = x, y
            cropping, button_down = True, True
            box_points[:] = [(x, y)]

        # Mouse is moving
        elif button_down and event == cv2.EVENT_MOUSEMOVE and cropping:
            image_copy = oriImage.copy()
            x_end, y_end = x, y
            cv2.rectangle(image_copy, box_points[0], (x, y), (0, 255, 0), 2)
            cv2.imshow("image", image_copy)

        # Mouse button up event
        elif event == cv2.EVENT_LBUTTONUP:
            button_down = False
            cropping = False
            box_points.append((x, y))
            x_end, y_end = x, y
            mX1, mY1, mX2, mY2 = x_start+3, y_start+3, x_end-3, y_end-3

            # Set background, crop image, and update display
            background = handle_bg_color()
            crop_image_with_bg()
            update_displayed_image()

    def selectMask():
        global oriImage, button_down

        def setup_mask_window():
            cv2.namedWindow("image")
            cv2.setMouseCallback("image", mask_crop)
            cv2.imshow("image", oriImage)

        button_down = False
        x_start, y_start, x_end, y_end = 0, 0, 0, 0  # Initialize coordinates
        mask_pic()  # Call external function for initial masking setup

        oriImage = cv2.imread('curImage.jpg').copy()  # Load and duplicate the current image
        setup_mask_window()  # Set up the window and callback for cropping

    def mouse_crop(event, x, y, flags, param):
        global x_start, y_start, x_end, y_end, cropping, button_down, oriImage, box_points

        # Draw a rectangle on a copy of the image and display it.
        def update_image_with_rectangle(image, start, end, color=(0, 255, 0), thickness=2):
            image_copy = image.copy()
            cv2.rectangle(image_copy, start, end, color, thickness)
            cv2.imshow("image", image_copy)

        cropDone = False

        if not button_down and event == cv2.EVENT_LBUTTONDOWN:
            # Start cropping
            x_start, y_start, x_end, y_end = x, y, x, y
            cropping = True
            button_down = True
            box_points = [(x, y)]

        elif button_down and event == cv2.EVENT_MOUSEMOVE and cropping:
            # Update rectangle as mouse moves
            x_end, y_end = x, y
            update_image_with_rectangle(oriImage, box_points[0], (x_end, y_end))

        elif event == cv2.EVENT_LBUTTONUP:
            # Finish cropping
            button_down = False
            box_points.append((x, y))
            cv2.rectangle(oriImage, box_points[0], box_points[1], (0, 255, 0), 2)
            cv2.imshow("image", oriImage)

            # Record final crop coordinates and set ROI
            x_end, y_end = x, y
            cropping = False
            refPoint = [(x_start + 3, y_start + 3), (x_end - 3, y_end - 3)]

            if len(refPoint) == 2:
                # Crop and save the region of interest
                roi = oriImage[refPoint[0][1]:refPoint[1][1], refPoint[0][0]:refPoint[1][0]]
                cv2.imshow("Cropped", roi)

                # Prompt for template name and save the cropped image
                template_name = simpledialog.askstring(title="Teach Vision Object", prompt="Save Object As:")
                if template_name:
                    cv2.imwrite(f"{template_name}.jpg", roi)
                cv2.destroyAllWindows()
                updateVisOp()

    def selectTemplate():
        global oriImage, button_down

        button_down = False
        x_start = y_start = x_end = y_end = 0

        image = cv2.imread("curImage.jpg")
        oriImage = image.copy()

        cv2.namedWindow("image")
        cv2.setMouseCallback("image", mouse_crop)
        cv2.imshow("image", image)

    def snapFind():
        global selectedTemplate, BGavg

        take_pic()

        template = selectedTemplate.get()
        min_score = float(VisScoreEntryField.get()) * 0.01
        autoBGVal = int(autoBG.get())
        background = BGavg if autoBGVal == 1 else eval(VisBacColorEntryField.get())

        if autoBGVal == 1:
            VisBacColorEntryField.configure(state="normal")
            VisBacColorEntryField.delete(0, "end")
            VisBacColorEntryField.insert(0, str(BGavg))
            VisBacColorEntryField.configure(state="disabled")

        visFind(template, min_score, background)

    def rotate_image(img, angle, background):
        image_center = tuple(np.array(img.shape[1::-1]) / 2)
        rot_mat = cv2.getRotationMatrix2D(image_center, -angle, 1.0)
        return cv2.warpAffine(
            img, rot_mat, img.shape[1::-1],
            borderMode=cv2.BORDER_CONSTANT, borderValue=background, flags=cv2.INTER_LINEAR
        )

    def visFind(template, min_score, background):
        global xMMpos, yMMpos, autoBG

        def set_background():
            if background == "Auto":
                background_val = BGavg
                VisBacColorEntryField.configure(state='enabled')
                VisBacColorEntryField.delete(0, 'end')
                VisBacColorEntryField.insert(0, str(BGavg))
                VisBacColorEntryField.configure(state='disabled')
                return background_val
            return eval(VisBacColorEntryField.get())

        def rotate_and_match_template(angle_step, method):
            best_score, best_angle, best_loc, best_dims = 0, 0, (0, 0), (0, 0)
            for angle in range(0, 360, angle_step):
                rotated_template = rotate_image(img2, angle, background)
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
            if pick180.get() == "1":
                angle += -180 if angle > 90 else (180 if angle < -90 else 0)
            limit = J6PosLim if angle > 0 else J6NegLim
            if pickClosest.get() == "0" and abs(angle) > limit:
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
            vid_lbl.imgtk = ImageTk.PhotoImage(image=img_resized)
            vid_lbl.configure(image=vid_lbl.imgtk)

        def fail_status():
            cv2.rectangle(img_copy, (5, 5), (635, 475), (255, 0, 0), 5)
            update_display()
            for field in [VisRetScoreEntryField, VisRetAngleEntryField, VisRetXpixEntryField, VisRetYpixEntryField]:
                field.delete(0, 'end')
                field.insert(0, "NA" if field != VisRetScoreEntryField else str(round(score * 100, 2)))
            return "fail"

        def process_match_success(score, angle, loc, dims):
            angle = normalize_angle(angle)
            xPos, yPos = int(loc[1] + dims[1] / 2), int(loc[0] + dims[0] / 2)
            draw_alignment_lines(angle, (xPos, yPos))
            update_display()
            
            fields_data = [
                (VisRetScoreEntryField, str(round(score * 100, 2))),
                (VisRetAngleEntryField, str(angle)),
                (VisRetXpixEntryField, str(xPos)),
                (VisRetYpixEntryField, str(yPos)),
                (VisRetXrobEntryField, str(round(xMMpos, 2))),
                (VisRetYrobEntryField, str(round(yMMpos, 2))),
            ]
            for field, data in fields_data:
                field.delete(0, 'end')
                field.insert(0, data)
            viscalc()
            return "pass"

        background = set_background()
        img1 = cv2.imread('curImage.jpg')
        img2 = cv2.imread(template)
        img_copy = img1.copy()
        method = cv2.TM_CCOEFF_NORMED
        fullRotVal = int(fullRot.get())

        if fullRotVal == 0:
            score, angle, loc, dims = refine_angle_search(method)
        else:
            score, angle, loc, dims = rotate_and_match_template(1, method)

        if score < min_score:
            return fail_status()
        else:
            return process_match_success(score, angle, loc, dims)
    
    def updateVisOp():
        global selectedTemplate
        selectedTemplate = StringVar()

        folder = os.path.dirname(sys.executable) if getattr(sys, 'frozen', False) else os.path.dirname(os.path.realpath(__file__))
        
        filelist = [fname for fname in os.listdir(folder) if fname.endswith('.jpg')]

        # Create and place the dropdown menu with file options
        Visoptmenu = ttk.Combobox(
            tab6, textvariable=selectedTemplate, values=filelist, state='readonly')
        Visoptmenu.place(x=390, y=52)

        # Bind the selection event to update functionality
        Visoptmenu.bind("<<ComboboxSelected>>", VisOpUpdate)

    def VisOpUpdate(_):
        global selectedTemplate
        file = selectedTemplate.get()

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
        template_lbl.imgtk = imgtk
        template_lbl.configure(image=imgtk)

    def zeroBrCn():
        global mX1, mY1, mX2, mY2

        # Set default coordinates and reset sliders
        mX1, mY1 = 0, 0
        mX2, mY2 = 640, 480
        VisBrightSlide.set(0)
        VisContrastSlide.set(0)

        take_pic()

    def VisUpdateBriCon(_):
        take_pic()

    def motion(event):
        y = event.x
        x = event.y

        if x <= 240 and y <= 320:
            # Update top-left corner coordinates
            VisX1PixEntryField.delete(0, 'end')
            VisX1PixEntryField.insert(0, x)
            VisY1PixEntryField.delete(0, 'end')
            VisY1PixEntryField.insert(0, y)
        elif x > 240:
            # Update bottom-right X coordinate
            VisX2PixEntryField.delete(0, 'end')
            VisX2PixEntryField.insert(0, x)
        elif y > 320:
            # Update bottom-right Y coordinate
            VisY2PixEntryField.delete(0, 'end')
            VisY2PixEntryField.insert(0, y)

    def checkAutoBG():
        autoBGVal = int(autoBG.get())
        # Disable or enable VisBacColorEntryField based on autoBG value
        state = 'disabled' if autoBGVal == 1 else 'enabled'
        VisBacColorEntryField.configure(state=state)

    # GCODE defs #

    def gcodeFrame():
        # Create and place the CTkFrame
        gcodeframe = ctk.CTkFrame(tab7)
        gcodeframe.place(x=300, y=10)

        # Set up the CTkScrollbar
        scrollbar = ctk.CTkScrollbar(gcodeframe)
        scrollbar.pack(side=ctk.RIGHT, fill=ctk.Y)

        # Configure the CTkListbox (if `customtkinter` lacks a CTkListbox, you may have to fall back to Listbox)
        tab7.gcodeView = ctk.CTkTextbox(gcodeframe, width=105, height=46, yscrollcommand=scrollbar.set)
        tab7.gcodeView.bind('<<ListboxSelect>>', gcodeViewselect)
        tab7.gcodeView.pack()
        
        # Configure the scrollbar to scroll the Listbox
        scrollbar.configure(command=tab7.gcodeView.yview)

        # Brief delay to allow the interface to update
        time.sleep(0.1)

    def gcodeViewselect(e):
        # Get the selected row in the gcodeView Listbox
        gcodeRow = tab7.gcodeView.curselection()[0]
        
        # Update the GcodCurRowEntryField with the selected row index
        GcodCurRowEntryField.delete(0, 'end')
        GcodCurRowEntryField.insert(0, gcodeRow)

    def loadGcodeProg():
        # Set file types for the file dialog
        filetypes = (('G-code files', '*.gcode *.nc *.ngc *.cnc *.tap'),
                    ('Text files', '*.txt'))

        # Open file dialog and get the selected file path
        filename = fd.askopenfilename(title='Open files', initialdir='/', filetypes=filetypes)
        if not filename:
            return  # Exit if no file is selected

        # Update GcodeProgEntryField with the selected filename
        GcodeProgEntryField.delete(0, 'end')
        GcodeProgEntryField.insert(0, filename)

        # Clear the current contents of gcodeView
        tab7.gcodeView.delete(0, END)
        
        # Open and read the G-code file
        with open(filename, "rb") as gcodeProg:
            previtem = b""
            for item in gcodeProg:
                # Remove comments from each line, if present
                commentIndex = item.find(b";")
                item = item[:commentIndex].strip() + b" "

                # Insert only unique lines
                if item != previtem:
                    tab7.gcodeView.insert(END, item)
                previtem = item

        # Configure scrollbar for gcodeView
        gcodescrollbar.config(command=tab7.gcodeView.yview)

    def SetGcodeStartPos():
        # List of entry fields and corresponding position variables
        entry_fields = [
            (GC_ST_E1_EntryField, XcurPos),
            (GC_ST_E2_EntryField, YcurPos),
            (GC_ST_E3_EntryField, ZcurPos),
            (GC_ST_E4_EntryField, RzcurPos),
            (GC_ST_E5_EntryField, RycurPos),
            (GC_ST_E6_EntryField, RxcurPos),
            (GC_ST_WC_EntryField, WC)
        ]

        # Update each entry field with the corresponding position value
        for entry_field, pos_value in entry_fields:
            entry_field.delete(0, 'end')
            entry_field.insert(0, str(pos_value))

    def MoveGcodeStartPos():
        # Calculate positions
        positions = {
            "X": float(GC_ST_E1_EntryField.get()) + float(GC_SToff_E1_EntryField.get()),
            "Y": float(GC_ST_E2_EntryField.get()) + float(GC_SToff_E2_EntryField.get()),
            "Z": float(GC_ST_E3_EntryField.get()) + float(GC_SToff_E3_EntryField.get()),
            "Rz": float(GC_ST_E4_EntryField.get()) + float(GC_SToff_E4_EntryField.get()),
            "Ry": float(GC_ST_E5_EntryField.get()) + float(GC_SToff_E5_EntryField.get()),
            "Rx": float(GC_ST_E6_EntryField.get()) + float(GC_SToff_E6_EntryField.get()),
            "J7": J7PosCur,
            "J8": J8PosCur,
            "J9": J9PosCur,
        }

        # Motion parameters
        speed_params = {
            "speedPrefix": "Sm",
            "Speed": "25",
            "ACCspd": "10",
            "DECspd": "10",
            "ACCramp": "100",
            "WC": GC_ST_WC_EntryField.get(),
        }

        # Loop mode
        loop_mode = "".join(
            str(stat.get())
            for stat in [J1OpenLoopStat, J2OpenLoopStat, J3OpenLoopStat, J4OpenLoopStat, J5OpenLoopStat, J6OpenLoopStat]
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
        cmdSentEntryField.delete(0, 'end')
        cmdSentEntryField.insert(0, command)
        ser.write(command.encode())
        ser.flushInput()
        time.sleep(0.1)
        response = ser.readline().strip().decode('utf-8')

        if response.startswith("E"):
            ErrorHandler(response)
        else:
            displayPosition(response)

    def GCstepFwd():
        # Update G-Code status
        GCalmStatusLab.config(text="GCODE READY", style="OK.TLabel")
        GCexecuteRow()

        # Get the currently selected row and total rows
        selected_row = tab7.gcodeView.curselection()[0]
        total_rows = tab7.gcodeView.index('end')

        # Update colors for executed, current, and pending rows
        for row in range(0, selected_row):
            tab7.gcodeView.itemconfig(row, {'fg': 'dodger blue'})
        tab7.gcodeView.itemconfig(selected_row, {'fg': 'blue2'})
        for row in range(selected_row + 1, total_rows):
            tab7.gcodeView.itemconfig(row, {'fg': 'black'})

        # Update selection for the next row
        tab7.gcodeView.selection_clear(0, END)
        next_row = selected_row + 1
        tab7.gcodeView.select_set(next_row)

        # Update the current row display field
        try:
            GcodCurRowEntryField.delete(0, 'end')
            GcodCurRowEntryField.insert(0, next_row)
        except Exception:  # Fallback in case of an error
            GcodCurRowEntryField.delete(0, 'end')
            GcodCurRowEntryField.insert(0, "---")

    def GCdelete():
        filename = GcodeFilenameField.get()

        # Validate input
        if not filename:
            messagebox.showwarning("Warning", "Please enter a filename")
            return

        # Prepare and send the delete command
        full_filename = f"{filename}.txt"
        command = f"DGFn{full_filename}\n"
        cmdSentEntryField.delete(0, 'end')
        cmdSentEntryField.insert(0, command)
        ser.write(command.encode())
        ser.flushInput()
        time.sleep(0.1)

        # Process response
        response = ser.readline().strip().decode('utf-8')
        if response.startswith('E'):
            ErrorHandler(response)
            return

        # Handle successful or failed deletion
        if response == "P":
            GCalmStatusLab.config(
                text=f"{full_filename} has been deleted", style="OK.TLabel")
            GCread("no")
        elif response == "F":
            GCalmStatusLab.config(
                text=f"{full_filename} was not found", style="Alarm.TLabel")

    def GCread(status):
        # Prepare and send the command
        command = "RG\n"
        cmdSentEntryField.delete(0, 'end')
        cmdSentEntryField.insert(0, command)
        ser.write(command.encode())
        ser.flushInput()
        time.sleep(0.1)

        # Receive and process the response
        response = ser.readline().strip().decode('utf-8')
        if response.startswith('E'):
            ErrorHandler(response)
            return

        # Update status if files are found
        if status == "yes":
            GCalmStatusLab.configure(text="FILES FOUND ON SD CARD:", fg_color="green")  # Updated for CTkLabel

        # Update the G-code program view
        GcodeProgEntryField.delete(0, 'end')
        tab7.gcodeView.delete(0, ctk.END)
        for value in response.split(","):
            tab7.gcodeView.insert(ctk.END, value)
        tab7.gcodeView.pack()
        gcodescrollbar.configure(command=tab7.gcodeView.yview)

    def GCplay():
        filename = GcodeFilenameField.get().strip()
        
        if not filename:
            messagebox.showwarning("Warning", "Please enter a valid filename.")
            return
        
        GCplayProg(filename)

    def GCplayProg(filename):
        if not filename.strip():
            GCalmStatusLab.config(text="No G-code file specified", style="Alarm.TLabel")
            return

        GCalmStatusLab.config(text=f"Running G-code File: {filename}", style="OK.TLabel")

    def GCplayProg(Filename):
        GCalmStatusLab.config(text="GCODE FILE RUNNING", style="OK.TLabel")

        def GCthreadPlay():
            global estopActive

            # Build the command and update UI fields
            Fn = Filename + ".txt"
            command = "PG" + "Fn" + Fn + "\n"
            cmdSentEntryField.delete(0, 'end')
            cmdSentEntryField.insert(0, command)

            # Send the command
            ser.write(command.encode())
            ser.flushInput()
            time.sleep(.1)

            # Process the response
            response = str(ser.readline().strip(), 'utf-8')
            if response[:1] == 'E':
                ErrorHandler(response)
            else:
                displayPosition(response)

                # Update status label based on estop state
                if estopActive == TRUE:
                    GCalmStatusLab.config(
                        text="Estop Button was Pressed", style="Alarm.TLabel")
                else:
                    GCalmStatusLab.config(
                        text="GCODE FILE COMPLETE", style="Warn.TLabel")

        # Start the process in a separate thread
        GCplay = threading.Thread(target=GCthreadPlay)
        GCplay.start()

    def GCconvertProg():
        if GcodeProgEntryField.get() == "":
            messagebox.showwarning("warning", "Please Load a Gcode Program")
            return
        if GcodeFilenameField.get() == "":
            messagebox.showwarning("warning", "Please Enter a Filename")
            return

        # Prepare command and update UI fields
        Filename = GcodeFilenameField.get() + ".txt"
        command = "DG" + "Fn" + Filename + "\n"
        cmdSentEntryField.delete(0, 'end')
        cmdSentEntryField.insert(0, command)
        ser.write(command.encode())
        ser.flushInput()
        time.sleep(.1)
        response = str(ser.readline().strip(), 'utf-8')
        last = tab7.gcodeView.index('end')
        for row in range(0, last):
            tab7.gcodeView.itemconfig(row, {'fg': 'black'})

        def GCthreadProg():
            global GCrowinproc, GCstopQueue, splineActive, prevxVal, prevyVal, prevzVal
            prevxVal, prevyVal, prevzVal = 0, 0, 0
            GCstopQueue, splineActive = "0", "0"

            try:
                GCselRow = tab7.gcodeView.curselection()[0]
                if GCselRow == 0:
                    GCselRow = 1
            except:
                GCselRow = 1
                tab7.gcodeView.selection_clear(0, END)
                tab7.gcodeView.select_set(GCselRow)

            tab7.GCrunTrue = 1

            while tab7.GCrunTrue == 1:
                if tab7.GCrunTrue == 0:
                    GCalmStatusLab.config(
                        text="GCODE CONVERSION STOPPED", style="Alarm.TLabel")
                    break

                GCalmStatusLab.config(
                    text="GCODE CONVERSION RUNNING", style="OK.TLabel")

                GCrowinproc = 1
                GCexecuteRow()

                while GCrowinproc == 1:
                    time.sleep(.1)

                try:
                    GCselRow = tab7.gcodeView.curselection()[0]
                    tab7.gcodeView.itemconfig(GCselRow, {'fg': 'blue2'})
                    tab7.gcodeView.selection_clear(0, END)
                    GCselRow += 1
                    tab7.gcodeView.select_set(GCselRow)
                    GcodCurRowEntryField.delete(0, 'end')
                    GcodCurRowEntryField.insert(0, GCselRow)
                except:
                    GcodCurRowEntryField.delete(0, 'end')
                    GcodCurRowEntryField.insert(0, "---")
                    tab7.GCrunTrue = 0
                    GCalmStatusLab.config(
                        text="GCODE CONVERSION STOPPED", style="Alarm.TLabel")

        GCt = threading.Thread(target=GCthreadProg)
        GCt.start()

    def GCstopProg():
        global cmdType, splineActive, GCstopQueue, moveInProc
        tab7.GCrunTrue = 0
        GCalmStatusLab.config(text="GCODE CONVERSION STOPPED", style="Alarm.TLabel")

        if splineActive == 1:
            splineActive = "0"
            if GCstopQueue == "1":
                GCstopQueue = "0"
                stop()

            if moveInProc == 1:
                moveInProc = 2

            command = "SS\n"
            cmdSentEntryField.delete(0, 'end')
            cmdSentEntryField.insert(0, command)
            ser.write(command.encode())
            ser.flushInput()
            response = str(ser.readline().strip(), 'utf-8')

            if response[:1] == 'E':
                ErrorHandler(response)
            else:
                displayPosition(response)

    def GCexecuteRow():
        # Current position variables
        global J1AngCur, J2AngCur, J3AngCur, J4AngCur, J5AngCur, J6AngCur

        # State and status flags
        global calStat, GCrowinproc, LineDist, moveInProc, splineActive, stopQueue

        # Coordinate and value tracking
        global Xv, Yv, Zv, prevxVal, prevyVal, prevzVal, xVal, yVal, zVal

        # Speed and configuration variables
        global commandCalc, gcodeSpeed, inchTrue

        def parse_coordinate(command, axis, default_val):
            if axis in command:
                value = command[command.find(axis) + 1:]
                value = value[:value.find(" ")] if " " in value else value
                value = str(round(float(value), 3))
                if inchTrue:
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
            WC = GC_ST_WC_EntryField.get()
            LoopMode = "111111"
            Filename = GcodeFilenameField.get() + ".txt"
            return (
                f"WCX{xVal}Y{yVal}Z{zVal}Rz{rzVal}Ry{ryVal}Rx{rxVal}J7{J7Val}J8{J8PosCur}J9{J9PosCur}"
                f"Sm{speed}Ac{ACCspd}Dc{DECspd}Rm{ACCramp}Rnd{Rounding}W{WC}Lm{LoopMode}Fn{Filename}\n"
            )

        GCstartTime = time.time()
        GCselRow = tab7.gcodeView.curselection()[0]
        tab7.gcodeView.see(GCselRow + 2)
        command = tab7.gcodeView.get(tab7.gcodeView.curselection()[0]).decode()
        cmdType, subCmd = command[:1], command[1:command.find(" ")].rstrip()

        if cmdType == "F":
            gcodeSpeed = command[command.find("F") + 1:]

        elif cmdType == "G":
            if subCmd in {"20", "21"}:
                inchTrue = subCmd == "20"

            elif subCmd in {"90", "91", "28"}:
                xVal, yVal, zVal = [
                    str(float(eval(f"GC_ST_E{i}_EntryField.get()")) + float(eval(f"GC_SToff_E{i}_EntryField.get()")))
                    for i in range(1, 4)
                ]
                rzVal, ryVal, rxVal = [
                    str(float(eval(f"GC_ST_E{i}_EntryField.get()")) + float(eval(f"GC_SToff_E{i}_EntryField.get()")))
                    for i in range(4, 7)
                ]
                command = create_gcode_command(xVal, yVal, zVal, rzVal, ryVal, rxVal, str(J7PosCur), "25")
                cmdSentEntryField.delete(0, 'end')
                cmdSentEntryField.insert(0, command)
                ser.write(command.encode())
                ser.flushInput()
                time.sleep(.1)
                response = str(ser.readline().strip(), 'utf-8')
                if response.startswith('E'):
                    ErrorHandler(response)
                    GCstopProg()
                    tab7.GCrunTrue = 0
                    GCalmStatusLab.config(text="UNABLE TO WRITE TO SD CARD", style="Alarm.TLabel")
                else:
                    displayPosition(response)

            elif subCmd in {"0", "1"}:
                xVal = parse_coordinate(command, "X", XcurPos)
                yVal = parse_coordinate(command, "Y", YcurPos)
                zVal = parse_coordinate(command, "Z", ZcurPos)

                rzVal = parse_coordinate(command, "A", RzcurPos)
                ryVal = parse_coordinate(command, "B", RycurPos)
                rxVal = parse_coordinate(command, "C", RxcurPos)

                J7Val = parse_coordinate(command, "E", J7PosCur)

                speed = gcodeSpeed if subCmd == "1" else speedEntryField.get()
                command = create_gcode_command(xVal, yVal, zVal, rzVal, ryVal, rxVal, J7Val, speed)
                prevxVal, prevyVal, prevzVal = xVal, yVal, zVal
                cmdSentEntryField.delete(0, 'end')
                cmdSentEntryField.insert(0, command)
                ser.write(command.encode())
                ser.flushInput()
                time.sleep(.05)
                response = str(ser.readline().strip(), 'utf-8')
                if response.startswith('E'):
                    ErrorHandler(response)
                    tab7.GCrunTrue = 0
                    GCalmStatusLab.config(text="UNABLE TO WRITE TO SD CARD", style="Alarm.TLabel")
                else:
                    displayPosition(response)

        GCrowinproc = 0

    # Application styling defs #

    ## TAB 1 ##

    ## TAB 2 ##

    ## TAB 3 ##

    ## TAB 4 ##

    ## TAB 5 ##

    ## TAB 6 ##

    ## TAB 7 ##

    ## TAB 8 ##

    # Program Exit #

    def on_closing(self):
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

    def run(self):
        # Start the main loop
        self.root.mainloop()


## Run the application ##
if __name__ == "__main__":
    app = RobotArmApp()
    app.run()
