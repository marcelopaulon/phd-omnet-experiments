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
        @display("i=misc/sensor,p=8.75,116.41");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=17.03,204.54");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=67.92,39.33");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=178.63,99.38");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=136.85,179.99");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=181.67,277.66");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=80.62,311.47");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=89.94,251.76");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=145.36,388.23");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=84.71,399.13");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=194.13,198.42");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=259.50,199.00");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=271.01,356.09");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=292.47,292.12");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=320.45,220.03");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=400.14,216.33");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=482.28,265.28");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=520.81,188.82");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=590.95,224.32");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=588.66,163.14");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=588.51,327.59");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=567.53,402.89");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=634.52,436.65");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=698.03,425.12");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=749.33,487.46");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=674.42,340.80");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=547.04,462.13");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=585.63,549.32");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=667.79,516.69");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=663.44,613.51");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=746.18,591.78");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=807.67,639.85");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=877.80,480.88");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=923.48,538.29");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=970.54,597.64");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=1068.56,594.97");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=858.76,598.58");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=832.76,533.50");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=783.69,363.99");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=835.72,420.06");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=746.55,304.80");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=837.18,284.78");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=927.55,246.92");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=1006.03,188.94");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=943.09,131.87");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=1025.14,100.13");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=1064.92,169.93");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=898.21,185.20");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=848.49,353.83");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor,p=919.24,392.38");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor,p=1001.50,362.27");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor,p=1017.64,266.74");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor,p=1099.66,249.36");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor,p=1081.10,350.46");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor,p=1224.98,313.23");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor,p=1166.54,260.58");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor,p=1212.45,219.84");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor,p=956.56,309.25");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor,p=766.93,209.54");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor,p=751.06,133.12");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor,p=771.70,50.28");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor,p=685.35,49.28");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor,p=656.75,115.33");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor,p=825.87,142.07");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor,p=881.17,69.70");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor,p=1065.24,454.58");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor,p=1011.62,506.14");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor,p=952.92,664.23");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor,p=1147.07,553.36");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor,p=1154.70,685.95");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor,p=1139.27,625.61");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor,p=1221.24,611.12");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor,p=1270.43,652.21");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor,p=1277.40,586.88");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor,p=1339.95,609.04");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor,p=1355.48,545.02");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor,p=1412.48,510.62");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor,p=1491.08,517.31");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor,p=1559.78,547.27");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor,p=1581.91,463.31");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor,p=1613.89,389.02");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor,p=1694.17,371.08");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor,p=1691.36,308.03");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor,p=1764.85,350.43");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor,p=1786.44,271.20");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor,p=1678.68,211.17");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor,p=1613.54,208.78");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor,p=1531.38,223.26");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor,p=1504.30,131.44");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor,p=1429.61,242.72");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor,p=1452.68,181.84");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor,p=1568.24,47.44");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor,p=1598.32,136.88");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor,p=1737.61,183.11");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor,p=1812.99,205.23");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor,p=1585.08,300.12");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor,p=1847.42,283.50");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor,p=1857.55,150.18");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor,p=1952.76,154.52");
    }

}

