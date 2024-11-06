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

    def processWaitInputOn(self, command, ser):
        """Handle 'Wait Input ON' command for the given serial connection."""
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

    def processWaitInputOff(self, command, ser):
        """Handle 'Wait Input OFF' command for the given serial connection."""
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

    def processWaitTime(self, command):
        """Handle 'Wait Time' command."""
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

    def processSetRegister(self, command):
        """Handles the 'Set Register' command."""
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

    def processSetPositionRegister(self, command):
        """Handles the 'Set Position Register' command."""
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

    def processCalibrate(self):
        """Handles the 'Calibrate' command."""
        if self.moveInProc:
            self.moveInProc = 2
        calRobotAll()
        if calStat == 0:
            stopProg()

    def processToolS(self, command):
        """Process the Tool S command, send it to the device, handle errors, and display position data."""

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

    def handleMoveVCommand(self, command):
        """Process the Move V command, send it to the device, handle errors, and display position."""

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
                def extract_value(response, label):
                    """Extracts a numeric value from the response string given a label prefix."""
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

    def handleMovePCommand(self, command):
        """Process the MoveP command, send it to the device, handle errors, and display position data."""

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

    def handleOffsPRCommand(self, command):
        """Process the OffsPR command, construct the command string, send it to the device, and handle response errors."""
        
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

        """Helper function to jog a joint in a specific direction."""

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

        """Helper function to handle jogging for a specific joint in a given direction."""
        
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

    # TODO: ln 4911

    # Calibration and save defs #

    

    # Vision defs #



    # GCODE defs #



    # Program Exit

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


# Run the application
if __name__ == "__main__":
    app = RobotArmApp()
    app.run()
