#!/usr/bin/env python3

import tkinter as tk

def show_detection_buttons(detections, class_names):
    selected_object = None

    def button_click(class_id):
        nonlocal selected_object
        selected_object = class_id
        root.destroy()

    root = tk.Tk()
    root.title("Select Object")

    for detection in detections:
        class_id = detection[5]
        class_name = class_names[class_id]
        button = tk.Button(root, text=class_name, command=lambda name=class_id: button_click(name))
        button.pack()

    root.mainloop()

    return selected_object

def show_detection_buttons_modified(detections, class_names):
    selected_object = None

    def button_click(class_id):
        nonlocal selected_object
        selected_object = class_id
        root.destroy()

    def on_escape(event):
        nonlocal selected_object
        selected_object = None
        root.destroy()

    root = tk.Tk()
    root.title("Select Object")

    # Binding Escape key to on_escape function
    root.bind('<Escape>', on_escape)

    for detection in detections:
        class_id = detection[5]
        class_name = class_names[class_id]
        button = tk.Button(root, text=class_name, command=lambda name=class_id: button_click(name))
        button.pack()

    root.mainloop()

    return selected_object