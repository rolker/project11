#!/usr/bin/env python3

import sys
import os
import tkinter as tk
import tkinter.filedialog
import git

class RepoManagerApplication(tk.Frame):
    def __init__(self, master=None, repo_dir = None):
        super().__init__(master)
        self.master = master
        self.pack(fill=tk.BOTH, expand=1)
        
        self.root_dir = tk.StringVar()
        self.root_dir.trace("w",self.onRootDirChanged)
        
        self.create_widgets()
      
        if repo_dir is None:
            self.root_dir.set(os.path.abspath(os.path.dirname(sys.argv[0])+'/../'))
        else:
            self.root_dir.set(repo_dir)
        
    def create_widgets(self):
        self.root_dir_frame = tk.LabelFrame(self,text="Project root")
        self.root_dir_frame.pack(side="top", fill=tk.X)
        self.root_dir_entry = tk.Entry(self.root_dir_frame,width=50,textvariable=self.root_dir)
        self.root_dir_entry.pack(side="left", fill=tk.X, expand=1)
        self.select_dir_button = tk.Button(self.root_dir_frame,text="...",command=self.onDirButtonPressed)
        self.select_dir_button.pack(side="right")
        
        self.button_bar = tk.Frame(self)
        self.button_bar.pack()
        
        tk.Button(self.button_bar, text='Select All', command=self.onSelectAll).pack(side='left')
        tk.Button(self.button_bar, text='Select None', command=self.onSelectNone).pack(side='left')

        tk.Button(self.button_bar, text='Set origin HTTPS', command=self.onSetOriginHTTPS).pack(side='left')
        tk.Button(self.button_bar, text='Set origin SSH', command=self.onSetOriginSSH).pack(side='left')

        custom_remote_frame = tk.LabelFrame(self.button_bar,text="Remote (url ex: https://github.com/CCOMJHC/)")
        custom_remote_frame.pack(side='left')

        tk.Label(custom_remote_frame, text="label:").pack(side='left')
        self.custom_remote_label = tk.Entry(custom_remote_frame)
        self.custom_remote_label.pack(side='left')

        tk.Label(custom_remote_frame, text="url:").pack(side='left')
        self.custom_remote_url = tk.Entry(custom_remote_frame)
        self.custom_remote_url.pack(side='left')
        
        tk.Button(custom_remote_frame, text='Set/Update', command=self.onSetUpdateRemote).pack(side='left')
        
        
        self.content_frame = tk.Frame(self)
        self.content_frame.pack(side = 'top', fill=tk.BOTH, expand=1)
        
        container = tk.Frame(self.content_frame)
        canvas = tk.Canvas(container)
        scrollbar = tk.Scrollbar(container, orient='vertical', command=canvas.yview)
        
        self.repos_frame = tk.LabelFrame(canvas,text="Repositories")
        self.repos_frame.bind('<Configure>', lambda e: canvas.configure(scrollregion=canvas.bbox('all')))
        #self.repos_frame.pack(side="left", fill=tk.BOTH, expand=1)
        
        canvas.create_window((0,0), window=self.repos_frame, anchor='nw')
        canvas.configure(yscrollcommand=scrollbar.set)
        
        container.pack(side='left', fill='both', expand=True)
        canvas.pack(side='left', fill='both', expand=True)
        scrollbar.pack(side='right', fill='y')
        
        self.console = tk.Text(self.content_frame)
        self.console.pack(side="right", fill=tk.BOTH, expand=1)
        
    def onDirButtonPressed(self):
        self.root_dir.set(tk.filedialog.askdirectory())
        
    def onRootDirChanged(self, *args):
        for widget in self.repos_frame.winfo_children():
            widget.destroy()
        
        self.repos={}
        self.console.delete(1.0,tk.END)
        repo = git.Repo(self.root_dir.get())
        self.repos[''] = {'repo':repo}
        
        for sm in repo.submodules:
            self.repos[sm.name] = {'submodule':sm}
            sm_path = os.path.join(self.root_dir.get(),sm.path)
            self.repos[sm.name]['repo'] = git.Repo(sm_path)
            
        keys = list(self.repos.keys())
        keys.sort()
        for k in keys:
            self.repos[k]['selected'] = self.addRepo(self.repos_frame,k)
            
    def addRepo(self, parent, name):
        ret = {}

        frame = tk.Frame(parent)
        frame.pack(side='top', fill=tk.X, expand=1)

        var = tk.IntVar(value=1)
        if name == '':
            name = '(root)'
        
        cb = tk.Checkbutton(frame, text=name, variable = var, onvalue=1, offvalue=0)
        cb.pack(side='left')
        
        spacer = tk.Frame(frame)
        spacer.pack(side='left', fill=tk.X, expand=1)
        
        button = tk.Button(frame, text="Show Remotes", command= lambda: self.onShowRemotesPressed(name) )
        button.pack(side='right')
                            
        return var
    
    def onShowRemotesPressed(self, name):
        if name == '(root)':
            name = ''
        self.console.insert('end','--- '+name+'\n')
        for r in self.repos[name]['repo'].remotes:
            self.console.insert('end','\t'+r.name+': ')
            for url in r.urls:
                self.console.insert('end',url)
            self.console.insert('end','\n')
        self.console.insert('end','---\n')

    def onSelectAll(self):
        for k,v in self.repos.items():
            v['selected'].set(1)
    
    def onSelectNone(self):
        for k,v in self.repos.items():
            v['selected'].set(0)

    def setRemote(self,label,prefix):
        self.console.insert('end','Set/Update: '+label+'\n')
        for k,v in self.repos.items():
            if v['selected'].get() == 1:
                self.console.insert('end','\t'+k+': ')
                if 'submodule' in v:
                    name = os.path.split(v['submodule'].url)[1]
                else:
                    name = 'project11.git'
                new_url = os.path.join(prefix,name)
                try:
                    r = v['repo'].remote(label)
                    self.console.insert('end','update '+new_url+'\n')
                    r.set_url(new_url)
                except ValueError:
                    self.console.insert('end','set '+new_url+'\n')
                    v['repo'].create_remote(label,new_url)
                

    def onSetOriginHTTPS(self):
        self.setRemote('origin','https://github.com/CCOMJHC/')
    
    def onSetOriginSSH(self):
        self.setRemote('origin','git@github.com:CCOMJHC/')
        
    def onSetUpdateRemote(self):
        label = self.custom_remote_label.get()
        url = self.custom_remote_url.get()
        if label and url:
            self.setRemote(label,url)
        
        
        
    
        
        
root = tk.Tk()
root.title("Project11 Repository Manager")

repo = None
if len(sys.argv) > 1:
    repo = sys.argv[1]

app = RepoManagerApplication(root,repo)
app.mainloop()

        
