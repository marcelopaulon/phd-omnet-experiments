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

network DADCA_ACK_n100_a8000m2_lpnFn1_0
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
        @display("i=misc/sensor,p=62.81,15.56");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=153.30,165.39");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=220.44,144.97");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=222.74,210.38");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=297.48,237.25");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=291.85,139.21");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=270.32,60.54");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=147.07,44.46");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=338.67,87.26");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=401.74,191.08");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=466.50,257.85");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=379.99,261.64");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=391.42,352.99");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=308.98,405.43");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=286.32,319.81");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=200.62,325.19");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=347.29,452.42");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=426.02,455.15");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=389.97,522.81");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=451.92,526.06");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=124.10,312.69");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=106.00,389.09");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=46.02,330.46");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=2.60,389.33");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=53.12,452.06");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=81.93,540.38");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=3.14,528.53");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=85.04,619.38");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=89.19,684.75");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=155.80,671.70");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=230.80,407.57");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=179.65,261.70");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=192.68,480.82");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=113.63,455.81");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=490.85,197.72");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=457.68,124.06");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=486.66,30.08");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=564.08,84.98");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=622.95,114.15");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=626.97,38.75");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=589.99,183.85");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=586.41,250.62");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=639.32,299.96");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=657.26,203.26");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=713.87,243.36");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=685.76,110.25");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=763.48,60.55");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=840.44,4.06");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=841.62,66.48");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor,p=904.70,109.28");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor,p=829.07,145.01");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor,p=749.73,321.60");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor,p=689.84,337.12");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor,p=830.31,239.81");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor,p=843.95,389.47");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor,p=822.02,329.56");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor,p=912.04,265.03");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor,p=1001.09,284.88");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor,p=910.17,368.00");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor,p=973.44,374.15");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor,p=939.67,444.58");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor,p=853.02,486.27");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor,p=1011.88,512.43");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor,p=1016.95,423.23");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor,p=1092.54,457.40");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor,p=1158.82,412.58");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor,p=1217.43,391.79");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor,p=1177.69,341.80");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor,p=1252.80,325.63");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor,p=1322.51,373.39");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor,p=1203.52,190.93");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor,p=1190.24,272.72");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor,p=1314.10,284.74");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor,p=1301.47,485.28");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor,p=1391.98,267.98");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor,p=1381.09,333.51");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor,p=1450.50,359.77");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor,p=1372.16,191.29");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor,p=1305.17,172.62");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor,p=1471.48,119.65");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor,p=1337.84,73.17");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor,p=1417.64,59.09");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor,p=1224.94,104.20");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor,p=1523.49,331.71");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor,p=1589.54,285.32");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor,p=1477.83,246.54");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor,p=1639.75,248.63");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor,p=1729.20,280.91");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor,p=1688.17,366.98");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor,p=1604.20,388.88");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor,p=1538.44,251.19");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor,p=1429.98,436.08");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor,p=1443.87,503.46");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor,p=1500.71,439.17");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor,p=1551.14,476.42");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor,p=1564.06,571.77");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor,p=1633.06,568.82");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor,p=1657.42,455.54");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor,p=1641.33,677.47");
    }

}

