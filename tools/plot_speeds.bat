python plot_net_dump.py -v -n D:/Andre_Back_Up/a_Projects_Files/Local_Level_Routing/Prague_Zizkov/Inputs_Outputs/zizkov_flora.net.xml --min-color-value=0 --max-color-value=14 -b --no-ticks --xlim 1510,3700 --ylim 435,2815 --title "High Flow Speeds (m/s)" -i D:/Andre_Back_Up/a_Projects_Files/Local_Level_Routing/Prague_Zizkov/Inputs_Outputs/Necessary_Outputs/edge_traffic.high.TURNS.xml,D:/Andre_Back_Up/a_Projects_Files/Local_Level_Routing/Prague_Zizkov/Inputs_Outputs/Necessary_Outputs/edge_traffic.high.TURNS.xml -m speed,entered --output="D:/Andre_Back_Up/a_Projects_Files/Local_Level_Routing/Prague_Zizkov/Inputs_Outputs/Necessary_Outputs/speeds.high.TURNS.png" --colormap="winter"
python plot_net_dump.py -v -n D:/Andre_Back_Up/a_Projects_Files/Local_Level_Routing/Prague_Zizkov/Inputs_Outputs/zizkov_flora.net.xml --min-color-value=0 --max-color-value=14 -b --no-ticks --xlim 1510,3700 --ylim 435,2815 --title "Low Flow Speeds (m/s)" -i D:/Andre_Back_Up/a_Projects_Files/Local_Level_Routing/Prague_Zizkov/Inputs_Outputs/Necessary_Outputs/edge_traffic.low.TURNS.xml,D:/Andre_Back_Up/a_Projects_Files/Local_Level_Routing/Prague_Zizkov/Inputs_Outputs/Necessary_Outputs/edge_traffic.low.TURNS.xml -m speed,entered --output="D:/Andre_Back_Up/a_Projects_Files/Local_Level_Routing/Prague_Zizkov/Inputs_Outputs/Necessary_Outputs/speeds.low.TURNS.png" --colormap="winter"
python plot_net_dump.py -v -n D:/Andre_Back_Up/a_Projects_Files/Local_Level_Routing/Prague_Zizkov/Inputs_Outputs/zizkov_flora_closed.net.xml --min-color-value=0 --max-color-value=14 -b --no-ticks --xlim 1510,3700 --ylim 435,2815 --title "Mid Flow Speeds (m/s)" -i D:/Andre_Back_Up/a_Projects_Files/Local_Level_Routing/Prague_Zizkov/Inputs_Outputs/Necessary_Outputs/edge_traffic.mid.TURNS.xml,D:/Andre_Back_Up/a_Projects_Files/Local_Level_Routing/Prague_Zizkov/Inputs_Outputs/Necessary_Outputs/edge_traffic.mid.TURNS.xml -m speed,entered --output="D:/Andre_Back_Up/a_Projects_Files/Local_Level_Routing/Prague_Zizkov/Inputs_Outputs/Necessary_Outputs/speeds.mid.TURNS.png" --colormap="winter"
pause