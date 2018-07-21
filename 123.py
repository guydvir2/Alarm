from tkinter import *
import tkinter.filedialog, tkinter.messagebox
import os
from os import system


# the root window:
# def Sticky():
#     r = Tk()
#     r.option_add('*font', '{Helvetica} 11')
#     text = TextWidget(r, bg='#f9f3a9', wrap='word', undo=True)  # create subclass here (and
#     # call it text instead of t)
#     text.focus_set()
#     text.pack(fill='both', expand=1)
#     r.geometry('220x235')
#     r.title('Note')
#
#     m = tkinter.Menu(r)
#     m.add_command(label="+")  # , command=text.new_window)
#     m.add_command(label="Save")  # , command=text.save_file)
#     m.add_command(label="Save As" , command=text.save_file_as)
#     m.add_command(label="Open")  # , command=text.open_file)
#     r.config(menu=m)
#     r.mainloop()


def save_file_as(whatever=None):
    _filetypes = [('Text', '*.txt'), ('All files', '*'), ]
    filename = tkinter.filedialog.asksaveasfilename(defaultextension='.txt', filetypes=_filetypes)
    f = open(filename, 'w')
    # f.write(text.get('1.0', 'end'))
    f.close()
    tkinter.messagebox.showinfo('FYI', 'File Saved')


class TextWidget(Text):
    def __init__(self, *args, **kwargs):
        Text.__init__(self, *args, **kwargs)  # pass all args to superclass
        self.filename = ''
        self._filetypes = [('Text', '*.txt'), ('All files', '*'), ]


# Sticky()
save_file_as()