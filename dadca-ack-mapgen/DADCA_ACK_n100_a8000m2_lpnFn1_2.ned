package projeto;

import inet.common.geometry.common.SimpleGeographicCoordinateSystem;
import inet.environment.common.PhysicalEnvironment;
import inet.environment.ground.FlatGround;
import inet.node.inet.AdhocHost;
import inet.visualizer.integrated.IntegratedVisualizer;
import inet.mobility.contract.IMobility;
import inet.mobility.static.StationaryMobility;
import inet.networklayer.configurator.ipv4.Ipv4NetworkConfigurator;
import inet.node.ethernet.Eth100M;
import inet.node.inet.SensorNode;
import inet.node.inet.StandardHost;
import inet.node.inet.WirelessHost;
import inet.physicallayer.apskradio.packetlevel.ApskScalarRadioMedium;
import inet.physicallayer.ieee802154.bitlevel.Ieee802154UwbIrRadioMedium;
import inet.physicallayer.ieee802154.packetlevel.Ieee802154NarrowbandScalarRadioMedium;
import inet.physicallayer.ieee80211.packetlevel.Ieee80211ScalarRadioMedium;
import inet.visualizer.integrated.IntegratedMultiVisualizer;
import inet.visualizer.integrated.IntegratedVisualizer;
import inet.physicallayer.contract.packetlevel.IRadioMedium;

network DADCA_ACK_n100_a8000m2_lpnFn1_2
{
    parameters:
		int numUAVs = default(1);
    submodules:
        visualizer: IntegratedVisualizer {
            parameters:
                @display("p=100,50");
        }
        coordinateSystem: SimpleGeographicCoordinateSystem {
            parameters:
                @display("p=100,150");
        }
        
        environment: PhysicalEnvironment {
        }
        
        configurator: Ipv4NetworkConfigurator {
            @display("p=0,200");
        }
        radioMedium: <default("Ieee802154NarrowbandScalarRadioMedium")> like IRadioMedium {
            @display("p=200,0");
        }
        quads[numUAVs]: MobileNode {
			
        } 
        
        groundStation: GroundStation {
            @display("i=misc/sensor3");
        }
       
        
    sensors_1: MobileSensorNode {
        @display("i=misc/sensor,p=100.00,100.00");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor,p=171.36,84.84");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=248.93,64.57");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=259.88,160.29");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=309.16,236.25");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=331.29,321.69");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=413.15,280.00");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=483.09,288.28");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=525.61,245.20");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=477.23,192.97");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=540.71,160.88");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=381.24,147.91");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=329.11,62.90");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=317.05,429.07");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=354.26,379.77");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=270.78,481.06");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=340.85,534.47");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=427.19,517.79");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=522.63,506.77");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=601.26,455.61");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=530.07,588.22");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=608.13,674.64");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=544.88,733.19");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=455.63,749.61");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=388.73,787.98");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=392.10,858.17");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=318.96,878.88");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=481.66,680.72");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=476.38,835.26");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=511.94,886.44");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=526.34,949.69");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=615.15,908.60");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=616.53,830.93");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=529.32,1043.93");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=534.59,1142.31");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=618.73,1020.15");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=479.57,999.43");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=679.59,983.66");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=719.43,1063.48");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=444.47,1074.32");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=594.92,1185.93");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=452.12,1145.28");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=399.31,990.11");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=667.99,717.77");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=616.55,612.50");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=676.92,617.90");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=717.01,540.49");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=668.31,485.28");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=800.12,491.60");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=830.30,403.41");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor,p=798.90,574.01");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor,p=872.03,577.01");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor,p=821.00,641.58");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor,p=862.06,724.25");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor,p=847.53,802.67");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor,p=750.63,793.38");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor,p=761.29,710.96");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor,p=742.21,942.69");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor,p=804.08,1084.29");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor,p=816.29,987.28");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor,p=855.41,1157.31");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor,p=863.29,1029.02");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor,p=624.27,760.20");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor,p=732.56,878.46");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor,p=693.52,818.08");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor,p=908.91,844.24");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor,p=953.36,922.24");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor,p=979.38,829.69");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor,p=1034.36,902.97");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor,p=1129.39,907.63");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor,p=1097.15,835.32");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor,p=1075.40,963.53");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor,p=1088.28,1043.69");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor,p=1125.73,1108.56");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor,p=1166.99,1054.74");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor,p=1071.17,1201.74");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor,p=1122.62,1282.51");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor,p=1180.01,1341.31");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor,p=1123.05,1377.34");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor,p=1198.62,1420.76");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor,p=1230.10,1271.44");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor,p=1243.96,1211.90");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor,p=1182.97,1224.26");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor,p=1049.29,1128.55");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor,p=1022.47,1032.95");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor,p=970.47,1083.10");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor,p=946.49,1005.51");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor,p=907.56,1075.10");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor,p=1243.41,1037.71");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor,p=1323.45,1044.67");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor,p=1402.18,1092.76");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor,p=1353.09,1153.52");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor,p=1449.25,1158.37");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor,p=1502.18,1048.97");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor,p=1560.92,1066.33");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor,p=1577.31,1007.16");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor,p=1640.15,1033.46");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor,p=1569.82,1175.03");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor,p=1477.53,1234.42");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor,p=1618.94,1125.73");
    }

}

