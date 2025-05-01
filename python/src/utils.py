# -*- coding: utf-8 -*-
"""
Created on Sun Nov 10 17:37:13 2024

@author: ammar
"""

from pyKey.pyKey_windows import pressKey, releaseKey
import pyautogui
import time

def switch_tab():
    pyautogui.hotkey('alt', 'tab')
        
def unpause_game():
    pyautogui.press('p')
    
def set_camera_behind():
    pyautogui.hotkey('shift', '6')
    time.sleep(0.2)
    
    pressKey('LSHIFT')
    pressKey(',')
    
    time.sleep(0.6)
    
    releaseKey('LSHIFT')
    releaseKey(',')
    
    pressKey('UP')
    time.sleep(0.7)
    releaseKey('UP')
    
def set_camera_outside():
    pyautogui.hotkey('shift', '4')
    time.sleep(0.2)
    
    pressKey('LSHIFT')
    pressKey(',')
    
    time.sleep(0.6)
    
    releaseKey('LSHIFT')
    releaseKey(',')
    
    pressKey('UP')
    time.sleep(0.4)
    releaseKey('UP')
    
    pressKey('LEFT')
    time.sleep(1.6)
    releaseKey('LEFT')
    



