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

network DADCA_ACK_n100_a8000m2_lpnFn1_9
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
        @display("i=misc/sensor,p=161.06,134.55");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=237.28,42.94");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=190.35,2.38");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=227.76,185.87");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=290.65,99.68");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=53.46,163.38");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=157.60,217.46");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=291.62,188.45");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=324.46,265.07");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=297.83,349.00");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=356.55,410.82");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=359.56,508.50");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=231.96,321.27");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=349.35,122.23");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=30.27,223.28");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=94.80,265.93");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=110.44,363.50");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=138.22,423.23");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=106.38,476.11");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=198.44,399.23");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=373.25,182.09");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=435.50,171.40");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=262.20,435.77");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=472.02,541.01");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=443.95,484.32");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=402.94,616.00");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=335.81,585.02");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=378.29,684.35");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=296.96,689.12");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=209.99,722.35");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=237.93,776.83");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=239.43,859.35");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=265.92,935.14");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=204.86,967.36");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=221.95,1026.98");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=211.89,1113.58");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=183.16,1181.05");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=119.34,1237.04");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=47.26,1294.98");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=55.98,1376.06");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=108.88,1330.76");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=187.54,1387.45");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=260.19,1318.99");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=213.64,1253.99");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=165.31,1470.26");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=217.23,1506.18");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=287.76,1487.15");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=379.29,1520.24");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=465.76,1518.43");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor,p=441.09,1452.43");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor,p=442.46,1370.33");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor,p=445.94,1301.15");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor,p=510.45,1339.23");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor,p=542.21,1258.03");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor,p=624.21,1208.77");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor,p=535.44,1181.12");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor,p=465.95,1191.00");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor,p=686.31,1224.85");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor,p=759.88,1213.99");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor,p=722.55,1138.28");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor,p=808.10,1157.91");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor,p=849.01,1333.72");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor,p=784.04,1359.95");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor,p=725.44,1380.43");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor,p=698.45,1321.71");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor,p=633.33,1121.20");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor,p=519.04,1096.32");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor,p=621.96,1282.71");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor,p=382.95,1356.10");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor,p=327.74,1425.60");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor,p=515.26,1452.82");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor,p=569.81,1411.75");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor,p=635.02,1395.94");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor,p=812.07,1449.11");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor,p=894.96,1503.96");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor,p=829.71,1532.78");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor,p=762.88,1515.63");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor,p=692.36,1574.67");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor,p=721.42,1628.22");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor,p=663.30,1689.24");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor,p=605.20,1659.63");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor,p=604.72,1737.52");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor,p=601.15,1810.85");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor,p=524.58,1773.16");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor,p=539.59,1709.21");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor,p=687.24,1751.25");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor,p=776.67,1791.23");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor,p=821.46,1727.13");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor,p=866.24,1654.18");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor,p=947.01,1569.22");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor,p=913.35,1751.32");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor,p=844.14,1785.07");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor,p=935.19,1670.62");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor,p=979.04,1718.06");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor,p=1050.64,1738.01");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor,p=1046.33,1831.21");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor,p=995.57,1885.68");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor,p=941.18,1916.56");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor,p=966.17,1971.94");
    }

}

