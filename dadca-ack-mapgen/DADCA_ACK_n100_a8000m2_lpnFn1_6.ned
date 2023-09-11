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

network DADCA_ACK_n100_a8000m2_lpnFn1_6
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
        @display("i=misc/sensor,p=99.54,1.01");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=179.75,95.36");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=31.15,165.38");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=21.08,246.70");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=113.53,269.95");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=110.89,206.55");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=30.94,60.26");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=172.58,165.15");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=160.56,27.69");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=189.78,292.02");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=260.22,285.70");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=231.34,204.62");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=141.71,401.54");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=197.38,368.98");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=284.68,391.90");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=13.86,340.38");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=347.55,277.73");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=410.13,267.74");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=375.32,348.48");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=363.11,437.11");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=455.81,454.41");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=433.57,510.23");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=360.91,501.24");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=440.44,599.48");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=424.14,659.21");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=430.13,725.59");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=483.37,675.28");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=340.58,691.04");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=494.38,565.37");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=524.12,475.75");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=543.90,604.80");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=594.56,673.17");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=616.01,743.05");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=579.33,835.46");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=524.18,792.49");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=617.82,902.93");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=560.62,962.69");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=500.54,895.55");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=449.96,819.46");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=694.08,857.03");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=666.74,787.54");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=678.77,931.60");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=703.96,993.95");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=683.20,1067.80");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=771.66,1079.75");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=722.13,1120.13");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=777.82,989.60");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=784.06,899.65");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=767.06,838.42");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor,p=878.89,1053.62");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor,p=926.83,1101.44");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor,p=1002.68,1042.84");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor,p=863.83,1134.64");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor,p=880.85,1246.26");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor,p=834.70,1291.40");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor,p=993.09,1314.68");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor,p=1011.11,1248.00");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor,p=1013.49,1399.99");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor,p=1026.79,1460.04");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor,p=1040.51,1533.50");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor,p=967.80,1480.51");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor,p=906.79,1485.65");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor,p=902.23,1413.56");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor,p=878.31,1339.04");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor,p=955.88,1365.98");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor,p=1111.99,1489.31");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor,p=1153.42,1545.95");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor,p=1238.56,1585.28");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor,p=1225.11,1491.96");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor,p=1146.29,1363.98");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor,p=1087.12,1378.62");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor,p=1199.29,1315.73");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor,p=1191.72,1406.77");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor,p=1069.32,1266.20");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor,p=756.64,1219.79");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor,p=770.89,1289.15");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor,p=681.11,1258.29");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor,p=642.71,1205.96");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor,p=810.81,1364.28");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor,p=656.96,1368.04");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor,p=721.04,1347.07");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor,p=727.83,1419.42");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor,p=967.74,1173.65");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor,p=947.84,1242.85");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor,p=599.36,1260.83");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor,p=530.70,1289.18");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor,p=524.61,1377.56");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor,p=601.59,1408.65");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor,p=547.39,1467.99");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor,p=637.16,1307.75");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor,p=1145.07,1214.93");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor,p=1281.26,1667.40");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor,p=1334.61,1708.54");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor,p=1294.56,1503.04");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor,p=1188.37,1631.04");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor,p=1318.05,1437.82");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor,p=1381.22,1436.99");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor,p=1370.12,1518.38");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor,p=1333.82,1592.93");
    }

}

