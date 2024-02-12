import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.transforms import offset_copy
import scienceplots
plt.style.use('science')
import os
from matplotlib.ticker import ScalarFormatter
import numpy as np

import subprocess
from multiprocessing import Pool

plt.rcParams['text.usetex'] = False



PROTOCOLS = ['TcpCubic', 'TcpBbr', 'TcpBbr3']
BWS = [10]
DELAYS = [5, 10, 15, 20, 25, 30, 35, 40, 45]
QMULTS = [0.2, 1, 4]
RUNS = [1, 2, 3, 4, 5]
LOSSES=[0]

MAX_SIMULATIONS = 16

def run_simulation(params):
    protocol, mult, delay, bw, delay, mult, protocol, run = params
    command = "./ns3 run \"scratch/SimulatorScript.cc --stopTime=200 --flowStartOffset=100 --appendFlow={} --queueBDP={} --botLinkDelay={} --p2pLinkDelay=2.5 --path=fairnessafter100secs/bw{}/delay{}/qmult{}/flows2/{}/run{} --seed={}\"".format(protocol, mult, delay, bw, delay, mult, protocol, run, run)
    subprocess.run(command, shell=True, cwd='../')

pool = Pool(processes=MAX_SIMULATIONS)
params_list = [(protocol, mult, delay, bw, delay, mult, protocol, run)
               for protocol in PROTOCOLS
               for bw in BWS
               for delay in DELAYS
               for mult in QMULTS
               for run in RUNS]

pool.map(run_simulation, params_list)
pool.close()
pool.join()


for mult in QMULTS:
   data = []
   for protocol in PROTOCOLS:
     for bw in BWS:
        for delay in DELAYS:

           start_time = 110
           end_time = 190

           BDP_IN_BYTES = int(bw * (2 ** 20) * 2 * delay * (10 ** -3) / 8)
           BDP_IN_PKTS = BDP_IN_BYTES / 1460

           goodput_ratios_20 = []
           goodput_ratios_total = []

           for run in RUNS:
              PATH = 'fairnessafter100secs/bw%s/delay%s/qmult%s/flows2/%s/run%s' % (bw,delay,mult,protocol,run)
              if os.path.exists(PATH + '/TcpBbr30-goodput.csv') and os.path.exists(PATH + '/'+protocol+'1-goodput.csv'):
                receiver1_total = pd.read_csv(PATH + '/TcpBbr30-goodput.csv').reset_index(drop=True)
                receiver2_total = pd.read_csv(PATH + '/'+protocol+'1-goodput.csv').reset_index(drop=True)
                

                receiver1_total.columns = ['time', 'goodput1']
                receiver2_total.columns = ['time', 'goodput2']
                #  receiver1_total[0] = receiver1_total[0].apply(lambda x: int(float(x)))
                #  receiver2_total[0] = receiver2_total[0].apply(lambda x: int(float(x)))
                #print(receiver1_total)
                #print(receiver2_total)

                receiver1_total = receiver1_total[(receiver1_total['time'] > start_time) & (receiver1_total['time'] < end_time)]
                receiver2_total = receiver2_total[(receiver2_total['time'] > start_time) & (receiver2_total['time'] < end_time)]


                receiver1 = receiver1_total[receiver1_total['time'] >= end_time].reset_index(drop=True)
                receiver2 = receiver2_total[receiver2_total['time'] >= end_time].reset_index(drop=True)



                receiver1_total = receiver1_total.set_index('time')
                receiver2_total = receiver2_total.set_index('time')


                receiver1 = receiver1.set_index('time')
                receiver2 = receiver2.set_index('time')


                total = receiver1_total.join(receiver2_total, how='inner', lsuffix='1', rsuffix='2')[['goodput1', 'goodput2']]
              
                # total = total.dropna()
                # partial = partial.dropna()

                goodput_ratios_total.append(total.min(axis=1)/total.max(axis=1))
              else:
                avg_goodput = None
                std_goodput = None
                jain_goodput_20 = None
                jain_goodput_total = None
                print("Folder %s not found." % PATH)
           if len(goodput_ratios_total) > 0:
              goodput_ratios_total = pd.concat(goodput_ratios_total, axis=0)
              
              #print (len(goodput_ratios_total))
              #if len(goodput_ratios_20) > 0 and len(goodput_ratios_total) > 0:
              data_entry = [protocol, bw, delay, delay/10, mult, goodput_ratios_total.mean(), goodput_ratios_total.std()]
              
              data.append(data_entry)
   summary_data = pd.DataFrame(data,
                              columns=['protocol', 'bandwidth', 'delay', 'delay_ratio','qmult', 
                                       'goodput_ratio_total_mean', 'goodput_ratio_total_std'])
    # PROTOCOLS = ['TcpCubic', 'TcpBbr', 'TcpBbr3']
   TcpCubic_data = summary_data[summary_data['protocol'] == 'TcpCubic'].set_index('delay')
   TcpBbr_data = summary_data[summary_data['protocol'] == 'TcpBbr'].set_index('delay')
   TcpBbr3_data = summary_data[summary_data['protocol'] == 'TcpBbr3'].set_index('delay')

   LINEWIDTH = 0.15
   ELINEWIDTH = 0.75
   CAPTHICK = ELINEWIDTH
   CAPSIZE= 2

   fig, axes = plt.subplots(nrows=1, ncols=1,figsize=(3,1.2))
   ax = axes



   markers, caps, bars = ax.errorbar(TcpCubic_data.index*2, TcpCubic_data['goodput_ratio_total_mean'], yerr=TcpCubic_data['goodput_ratio_total_std'],marker='x',linewidth=LINEWIDTH, elinewidth=ELINEWIDTH, capsize=CAPSIZE, capthick=CAPTHICK, label='TcpCubic')
   [bar.set_alpha(0.5) for bar in bars]
   [cap.set_alpha(0.5) for cap in caps]
   markers, caps, bars = ax.errorbar(TcpBbr_data.index*2,TcpBbr_data['goodput_ratio_total_mean'], yerr=TcpBbr_data['goodput_ratio_total_std'],marker='^',linewidth=LINEWIDTH, elinewidth=ELINEWIDTH, capsize=CAPSIZE, capthick=CAPTHICK,label='TcpBbr')
   [bar.set_alpha(0.5) for bar in bars]
   [cap.set_alpha(0.5) for cap in caps]
   markers, caps, bars = ax.errorbar(TcpBbr3_data.index*2,TcpBbr3_data['goodput_ratio_total_mean'], yerr=TcpBbr3_data['goodput_ratio_total_std'],marker='+',linewidth=LINEWIDTH, elinewidth=ELINEWIDTH, capsize=CAPSIZE, capthick=CAPTHICK,label='TcpBbr3')
   [bar.set_alpha(0.5) for bar in bars]
   [cap.set_alpha(0.5) for cap in caps]

   ax.set(yscale='linear',xlabel='RTT (ms)', ylabel='Goodput Ratio', ylim=[-0.1,1.1])
   for axis in [ax.xaxis, ax.yaxis]:
       axis.set_major_formatter(ScalarFormatter())
   # ax.legend(loc=4,prop={'size': 6})
   # get handles
   handles, labels = ax.get_legend_handles_labels()
   # remove the errorbars
   handles = [h[0] for h in handles]

   legend = fig.legend(handles, labels,ncol=3, loc='upper center',bbox_to_anchor=(0.5, 1.08),columnspacing=0.8,handletextpad=0.5)
   # ax.grid()

   for format in ['png', 'pdf']:
      plt.savefig('goodput_ratio_async_intra_20_qsize%s.%s' % (mult, format), dpi=1080)
