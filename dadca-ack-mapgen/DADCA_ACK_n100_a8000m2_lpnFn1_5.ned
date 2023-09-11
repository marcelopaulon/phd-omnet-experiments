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

network DADCA_ACK_n100_a8000m2_lpnFn1_5
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
        @display("i=misc/sensor,p=27.48,151.90");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=53.38,217.20");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=124.93,159.83");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=107.24,282.20");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=151.27,328.70");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=210.65,376.50");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=257.08,421.06");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=207.68,482.38");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=353.36,447.72");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=385.14,501.33");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=342.80,347.33");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=282.93,292.87");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=220.95,246.52");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=282.18,191.35");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=247.23,136.06");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=241.52,73.59");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=301.37,37.89");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=314.15,105.74");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=177.40,11.21");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=428.03,81.37");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=417.57,166.73");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=359.90,187.43");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=427.79,285.52");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=464.82,235.21");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=368.70,250.16");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=572.05,134.01");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=635.21,136.69");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=646.27,47.08");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=742.61,53.23");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=734.33,175.84");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=693.75,113.11");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=669.71,194.47");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=734.39,240.40");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=784.10,317.99");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=821.56,249.11");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=875.43,183.09");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=887.27,108.42");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=998.48,84.99");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=928.35,63.55");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=1027.06,29.15");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=943.47,271.92");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=1074.29,252.11");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=1146.79,303.88");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=1215.03,254.31");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=1294.12,223.57");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=1215.60,179.73");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=1240.66,111.74");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=1241.16,48.17");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=1151.55,16.82");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor,p=1172.30,76.05");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor,p=1146.97,172.73");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor,p=1105.76,82.82");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor,p=1153.38,241.00");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor,p=1301.24,162.17");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor,p=1308.38,85.50");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor,p=1382.95,129.37");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor,p=1215.19,354.19");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor,p=1227.45,449.82");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor,p=1300.74,435.09");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor,p=1258.85,516.42");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor,p=1146.58,500.80");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor,p=1090.59,538.24");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor,p=1016.85,594.92");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor,p=975.84,667.32");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor,p=973.51,532.90");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor,p=996.82,740.86");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor,p=979.03,830.81");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor,p=924.04,777.47");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor,p=862.71,838.84");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor,p=1013.12,898.56");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor,p=959.34,972.08");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor,p=968.70,1069.50");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor,p=903.64,1076.36");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor,p=842.19,1098.54");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor,p=906.09,1140.32");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor,p=823.25,1183.09");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor,p=780.66,1264.23");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor,p=848.87,1272.09");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor,p=902.04,1308.55");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor,p=962.23,1250.58");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor,p=968.56,1171.63");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor,p=737.88,1204.65");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor,p=1054.98,1137.59");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor,p=1125.69,1138.06");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor,p=1114.97,1057.81");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor,p=1193.66,1024.12");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor,p=1195.48,939.45");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor,p=1258.84,1006.78");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor,p=1213.88,1093.48");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor,p=1122.60,971.64");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor,p=1030.61,1070.53");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor,p=1044.00,992.59");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor,p=827.68,955.26");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor,p=803.49,868.41");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor,p=798.55,795.30");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor,p=854.01,716.36");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor,p=687.30,746.51");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor,p=643.66,807.88");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor,p=583.81,885.56");
    }

}

