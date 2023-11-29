import matplotlib.pyplot as plt 
from matplotlib.lines import Line2D
import numpy as np

plt.ion()  # enable interactive drawing

class myPlot:
    ''' 
        Create each individual subplot.
    '''
    def __init__(self, ax,
                 xlabel='',
                 ylabel='',
                 title='',
                 legend=None):
        ''' 
            ax - This is a handle to the  axes of the figure
            xlable - Label of the x-axis
            ylable - Label of the y-axis
            title - Plot title
            legend - A tuple of strings that identify the data. 
                     EX: ("data1","data2", ... , "dataN")
        '''
        self.legend = legend
        self.ax = ax                  # Axes handle
        self.colors = ['b', 'g', 'r', 'c', 'm', 'y', 'b']
        # A list of colors. The first color in the list corresponds
        # to the first line object, etc.
        # 'b' - blue, 'g' - green, 'r' - red, 'c' - cyan, 'm' - magenta
        # 'y' - yellow, 'k' - black
        self.line_styles = ['-', '-', '--', '-.', ':']
        # A list of line styles.  The first line style in the list
        # corresponds to the first line object.
        # '-' solid, '--' dashed, '-.' dash_dot, ':' dotted

        self.line1 = []


        # Configure the axes
        self.ax.set_ylabel(ylabel)
        self.ax.set_xlabel(xlabel)
        self.ax.set_title(title)
        self.ax.grid(True)

        # Keeps track of initialization
        self.init = True   

    def update(self, time, data1,label1):
        ''' 
            Adds data to the plot.  
            time is a list, 
            data is a list of lists, each list corresponding to a line on the plot
        '''
        if self.init == True:  # Initialize the plot the first time routine is called
            for i in range(len(data1)):
                # Instantiate line object1 and add it to the axes
                self.line1.append(Line2D(time,
                                        data1[i],
                                        color='r',
                                        ls=self.line_styles[np.mod(i, len(self.line_styles) - 1)],
                                        label=label1))
                self.ax.add_line(self.line1[i])

            self.init = False
            # add legend 
            #self.ax.legend(handles = [self.line1[0],self.line2[0],self.line3[0]], loc = 'upper left')
            

        else: # Add new data to the plot
            # Updates the x and y data of each line.
            for i in range(len(self.line1)):
                self.line1[i].set_xdata(time)
                self.line1[i].set_ydata(data1[i])

        # Adjusts the axis to fit all of the data
        self.ax.relim()
        self.ax.autoscale()

class myPlot2:
    ''' 
        Create each individual subplot.
    '''
    def __init__(self, ax,
                 xlabel='',
                 ylabel='',
                 title='',
                 legend=None):
        ''' 
            ax - This is a handle to the  axes of the figure
            xlable - Label of the x-axis
            ylable - Label of the y-axis
            title - Plot title
            legend - A tuple of strings that identify the data. 
                     EX: ("data1","data2", ... , "dataN")
        '''
        self.legend = legend
        self.ax = ax                  # Axes handle
        self.colors = ['b', 'g', 'r', 'c', 'm', 'y', 'b']
        # A list of colors. The first color in the list corresponds
        # to the first line object, etc.
        # 'b' - blue, 'g' - green, 'r' - red, 'c' - cyan, 'm' - magenta
        # 'y' - yellow, 'k' - black
        self.line_styles = ['-', '-', '--', '-.', ':']
        # A list of line styles.  The first line style in the list
        # corresponds to the first line object.
        # '-' solid, '--' dashed, '-.' dash_dot, ':' dotted

        self.line1 = []
        self.line2 = []

        # Configure the axes
        self.ax.set_ylabel(ylabel)
        self.ax.set_xlabel(xlabel)
        self.ax.set_title(title)
        self.ax.grid(True)

        # Keeps track of initialization
        self.init = True   

    def update(self, time, data1, data2, label1, label2):
        ''' 
            Adds data to the plot.  
            time is a list, 
            data is a list of lists, each list corresponding to a line on the plot
        '''
        if self.init == True:  # Initialize the plot the first time routine is called
            for i in range(len(data1)):
                # Instantiate line object1 and add it to the axes
                self.line1.append(Line2D(time,
                                        data1[i],
                                        color='r',
                                        ls=self.line_styles[np.mod(i, len(self.line_styles) - 1)],
                                        label=label1))
                self.ax.add_line(self.line1[i])

            for i in range(len(data2)):
                # Instantiate line object2 and add it to the axes
                self.line2.append(Line2D(time,
                                        data2[i],
                                        color='b',
                                        ls=self.line_styles[np.mod(i, len(self.line_styles) - 1)],
                                        label=label2))
                self.ax.add_line(self.line2[i])

            self.init = False
            # add legend 
            self.ax.legend(handles = [self.line1[0],self.line2[0]], loc = 'upper left')
            

        else: # Add new data to the plot
            # Updates the x and y data of each line.
            for i in range(len(self.line1)):
                self.line1[i].set_xdata(time)
                self.line1[i].set_ydata(data1[i])
                self.line2[i].set_xdata(time)
                self.line2[i].set_ydata(data2[i])

        # Adjusts the axis to fit all of the data
        self.ax.relim()
        self.ax.autoscale()

class myPlot3:
    ''' 
        Create each individual subplot.
    '''
    def __init__(self, ax,
                 xlabel='',
                 ylabel='',
                 title='',
                 legend=None):
        ''' 
            ax - This is a handle to the  axes of the figure
            xlable - Label of the x-axis
            ylable - Label of the y-axis
            title - Plot title
            legend - A tuple of strings that identify the data. 
                     EX: ("data1","data2", ... , "dataN")
        '''
        self.legend = legend
        self.ax = ax                  # Axes handle
        self.colors = ['b', 'g', 'r', 'c', 'm', 'y', 'b']
        # A list of colors. The first color in the list corresponds
        # to the first line object, etc.
        # 'b' - blue, 'g' - green, 'r' - red, 'c' - cyan, 'm' - magenta
        # 'y' - yellow, 'k' - black
        self.line_styles = ['-', '-', '--', '-.', ':']
        # A list of line styles.  The first line style in the list
        # corresponds to the first line object.
        # '-' solid, '--' dashed, '-.' dash_dot, ':' dotted

        self.line1 = []
        self.line2 = []
        self.line3 = []

        # Configure the axes
        self.ax.set_ylabel(ylabel)
        self.ax.set_xlabel(xlabel)
        self.ax.set_title(title)
        self.ax.grid(True)

        # Keeps track of initialization
        self.init = True   

    def update(self, time, data1, data2, data3, label1, label2, label3):
        ''' 
            Adds data to the plot.  
            time is a list, 
            data is a list of lists, each list corresponding to a line on the plot
        '''
        if self.init == True:  # Initialize the plot the first time routine is called
            for i in range(len(data1)):
                # Instantiate line object1 and add it to the axes
                self.line1.append(Line2D(time,
                                        data1[i],
                                        color='r',
                                        ls=self.line_styles[np.mod(i, len(self.line_styles) - 1)],
                                        label=label1))
                self.ax.add_line(self.line1[i])

            for i in range(len(data2)):
                # Instantiate line object2 and add it to the axes
                self.line2.append(Line2D(time,
                                        data2[i],
                                        color='b',
                                        ls=self.line_styles[np.mod(i, len(self.line_styles) - 1)],
                                        label=label2))
                self.ax.add_line(self.line2[i])

            for i in range(len(data3)):
                # Instantiate line object3 and add it to the axes
                self.line3.append(Line2D(time,
                                        data3[i],
                                        color='g',
                                        ls=self.line_styles[np.mod(i, len(self.line_styles) - 1)],
                                        label=label3))
                self.ax.add_line(self.line3[i])

            self.init = False
            # add legend 
            self.ax.legend(handles = [self.line1[0],self.line2[0],self.line3[0]], loc = 'upper left')
            

        else: # Add new data to the plot
            # Updates the x and y data of each line.
            for i in range(len(self.line1)):
                self.line1[i].set_xdata(time)
                self.line1[i].set_ydata(data1[i])
                self.line2[i].set_xdata(time)
                self.line2[i].set_ydata(data2[i])
                self.line3[i].set_xdata(time)
                self.line3[i].set_ydata(data3[i])

        # Adjusts the axis to fit all of the data
        self.ax.relim()
        self.ax.autoscale()
           
class myPlot4:
    ''' 
        Create each individual subplot.
    '''
    def __init__(self, ax,
                 xlabel='',
                 ylabel='',
                 title='',
                 legend=None):
        ''' 
            ax - This is a handle to the  axes of the figure
            xlable - Label of the x-axis
            ylable - Label of the y-axis
            title - Plot title
            legend - A tuple of strings that identify the data. 
                     EX: ("data1","data2", ... , "dataN")
        '''
        self.legend = legend
        self.ax = ax                  # Axes handle
        self.colors = ['b', 'g', 'r', 'c', 'm', 'y', 'b']
        # A list of colors. The first color in the list corresponds
        # to the first line object, etc.
        # 'b' - blue, 'g' - green, 'r' - red, 'c' - cyan, 'm' - magenta
        # 'y' - yellow, 'k' - black
        self.line_styles = ['-', '-', '--', '-.', ':']
        # A list of line styles.  The first line style in the list
        # corresponds to the first line object.
        # '-' solid, '--' dashed, '-.' dash_dot, ':' dotted

        self.line1 = []
        self.line2 = []
        self.line3 = []
        self.line4 = []

        # Configure the axes
        self.ax.set_ylabel(ylabel)
        self.ax.set_xlabel(xlabel)
        self.ax.set_title(title)
        self.ax.grid(True)

        # Keeps track of initialization
        self.init = True   

    def update(self, time, data1, data2, data3, data4, label1, label2, label3, label4):
        ''' 
            Adds data to the plot.  
            time is a list, 
            data is a list of lists, each list corresponding to a line on the plot
        '''
        if self.init == True:  # Initialize the plot the first time routine is called
            for i in range(len(data1)):
                # Instantiate line object1 and add it to the axes
                self.line1.append(Line2D(time,
                                        data1[i],
                                        color='r',
                                        ls=self.line_styles[np.mod(i, len(self.line_styles) - 1)],
                                        label=label1))
                self.ax.add_line(self.line1[i])

            for i in range(len(data2)):
                # Instantiate line object2 and add it to the axes
                self.line2.append(Line2D(time,
                                        data2[i],
                                        color='b',
                                        ls=self.line_styles[np.mod(i, len(self.line_styles) - 1)],
                                        label=label2))
                self.ax.add_line(self.line2[i])

            for i in range(len(data3)):
                # Instantiate line object3 and add it to the axes
                self.line3.append(Line2D(time,
                                        data3[i],
                                        color='g',
                                        ls=self.line_styles[np.mod(i, len(self.line_styles) - 1)],
                                        label=label3))
                self.ax.add_line(self.line3[i])

            for i in range(len(data4)):
                # Instantiate line object3 and add it to the axes
                self.line4.append(Line2D(time,
                                        data4[i],
                                        color='c',
                                        ls=self.line_styles[np.mod(i, len(self.line_styles) - 1)],
                                        label=label4))
                self.ax.add_line(self.line4[i])

            self.init = False
            # add legend 
            self.ax.legend(handles = [self.line1[0],self.line2[0],self.line3[0],self.line4[0]], loc = 'upper left')
            

        else: # Add new data to the plot
            # Updates the x and y data of each line.
            for i in range(len(self.line1)):
                self.line1[i].set_xdata(time)
                self.line1[i].set_ydata(data1[i])
                self.line2[i].set_xdata(time)
                self.line2[i].set_ydata(data2[i])
                self.line3[i].set_xdata(time)
                self.line3[i].set_ydata(data3[i])
                self.line4[i].set_xdata(time)
                self.line4[i].set_ydata(data4[i])

        # Adjusts the axis to fit all of the data
        self.ax.relim()
        self.ax.autoscale()

class FlightCharts:
    def __init__(self):
        # Number of subplots = num_of_rows*num_of_cols
        self.num_rows = 2    # Number of subplot rows
        self.num_cols = 3    # Number of subplot columns
        figuresize = (10,8)

        # Crete figure and axes handles
        self.fig, self.ax = plt.subplots(self.num_rows, self.num_cols, sharex=True, figsize = figuresize)
        #self.fig.tight_layout(pad = 0.4, w_pad = 0.5, h_pad = 1.0)

        # Instantiate lists to hold the time and data histories
        self.time_history = []  # time

        self.pn_history = []  # reference position z_r
        self.pe_history = []  # position z
        self.pd_history = []  # angle theta
        self.hc_history = []

        self.phi_history=[]
        self.theta_history=[]
        self.psi_history=[]

        self.pn2_history = []  # reference position z_r
        self.pe2_history = []  # position z
        self.pd2_history = []  # angle theta
        self.hc2_history = []

        self.phi2_history=[]
        self.theta2_history=[]
        self.psi2_history=[]

        self.Va_history=[]
        self.Vac_history=[]

        self.Chi_history=[]
        self.Chic_history=[]

        self.Va2_history=[]
        self.Vac2_history=[]

        self.Chi2_history=[]
        self.Chic2_history=[]

        self.handle = []
        self.handle.append(myPlot4(self.ax[0,0], ylabel='(m)', xlabel= 'Time(s)'))
        self.handle.append(myPlot3(self.ax[0,1], ylabel='(deg)',xlabel= 'Time(s)',title = 'Flight Data'))
        self.handle.append(myPlot3(self.ax[0,2], ylabel='(m)',xlabel= 'Time(s)'))
        self.handle.append(myPlot3(self.ax[1,0], ylabel='(deg)',xlabel= 'Time(s)'))
        self.handle.append(myPlot4(self.ax[1,1], ylabel='(m/s)',xlabel= 'Time(s)'))
        self.handle.append(myPlot4(self.ax[1,2], ylabel='(deg)',xlabel= 'Time(s)'))


    def update(self,t,state, state2, Va, Va2, Chi, Chi2, Va_c, Va_c2, Chi_c, Chi_c2, h_c):

        pn = state[0,0]
        pe = state[1,0]
        pd = state[2,0]

        phi = np.rad2deg(state[6,0])
        theta = np.rad2deg(state[7,0])
        psi = np.rad2deg(state[8,0])

        pn2 = state2[0,0]
        pe2 = state2[1,0]
        pd2 = state2[2,0]

        phi2 = np.rad2deg(state2[6,0])
        theta2 = np.rad2deg(state2[7,0])
        psi2 = np.rad2deg(state2[8,0])

        self.time_history.append(t)  # time

        self.pn_history.append(pn)  # reference position z_r
        self.pe_history.append(pe)  # position z
        self.pd_history.append(-pd)
        self.hc_history.append(h_c)        

        self.phi_history.append(phi)
        self.theta_history.append(theta)
        self.psi_history.append(psi)

        self.pn2_history.append(pn2)  # reference position z_r
        self.pe2_history.append(pe2)  # position z
        self.pd2_history.append(-pd2)       

        self.phi2_history.append(phi2)
        self.theta2_history.append(theta2)
        self.psi2_history.append(psi2)        

        self.Va_history.append(Va)
        self.Vac_history.append(Va_c)

        self.Chi_history.append(np.rad2deg(Chi))
        self.Chic_history.append(np.rad2deg(Chi_c))

        self.Va2_history.append(Va2)
        self.Vac2_history.append(Va_c2)

        self.Chi2_history.append(np.rad2deg(Chi2))
        self.Chic2_history.append(np.rad2deg(Chi_c2))

        self.handle[0].update(self.time_history, [self.pd_history], [self.hc_history], [self.pe_history], [self.pn_history],'pd','Target h','pe','pn')
        self.handle[1].update(self.time_history, [self.phi_history], [self.theta_history], [self.psi_history],'phi','theta','psi')
        self.handle[2].update(self.time_history, [self.pn2_history],[self.pe2_history],[self.pd2_history],'pn2','pe2','pd2')
        self.handle[3].update(self.time_history, [self.phi2_history],[self.theta2_history],[self.psi2_history],'phi2','theta2','pis2')
        self.handle[4].update(self.time_history, [self.Va_history],[self.Vac_history], [self.Va2_history], [self.Vac2_history], 'Va', 'Target Va','Va2','Target Va2')
        self.handle[5].update(self.time_history, [self.Chi_history], [self.Chic_history], [self.Chi2_history], [self.Chic2_history],'chi', 'Target chi', 'chi2', 'Target chi2')
