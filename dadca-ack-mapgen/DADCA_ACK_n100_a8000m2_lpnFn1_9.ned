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
        @display("i=misc/sensor,p=169.25,67.41");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=258.31,62.98");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=203.83,132.52");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=274.10,136.32");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=329.96,187.16");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=346.42,251.28");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=404.39,318.09");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=356.59,400.15");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=421.94,384.70");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=292.30,381.76");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=274.49,472.94");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=288.12,563.84");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=343.74,594.64");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=428.04,551.41");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=409.91,610.18");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=249.26,622.99");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=198.65,680.81");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=200.31,754.41");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=136.63,732.86");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=191.36,497.94");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=117.24,497.81");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=56.38,516.94");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=23.20,458.29");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=147.85,570.26");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=137.48,656.54");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=296.52,661.52");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=267.54,718.87");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=254.93,811.92");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=335.52,799.00");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=408.22,846.83");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=467.61,829.89");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=443.03,759.37");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=494.76,702.77");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=543.81,742.15");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=591.20,821.74");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=653.51,824.84");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=673.61,906.74");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=719.22,822.36");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=651.28,755.86");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=599.67,695.22");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=738.70,735.08");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=771.89,854.05");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=812.66,693.53");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=582.15,903.15");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=619.84,993.34");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=550.53,973.85");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=725.31,664.41");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=835.24,617.04");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=817.18,519.99");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor,p=644.06,632.76");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor,p=567.89,639.85");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor,p=553.41,541.46");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor,p=618.64,538.30");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor,p=567.56,403.56");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor,p=649.90,430.47");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor,p=561.35,467.47");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor,p=721.70,461.08");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor,p=711.18,394.48");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor,p=765.43,357.14");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor,p=825.70,327.17");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor,p=744.67,287.99");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor,p=697.29,223.40");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor,p=647.51,276.55");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor,p=583.82,237.75");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor,p=522.28,278.39");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor,p=534.89,346.56");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor,p=419.48,230.59");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor,p=116.27,413.46");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor,p=160.27,325.11");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor,p=231.21,338.68");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor,p=200.53,422.49");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor,p=26.68,303.64");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor,p=40.28,362.40");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor,p=343.66,520.23");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor,p=107.10,259.88");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor,p=205.15,243.67");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor,p=334.40,28.12");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor,p=387.77,114.95");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor,p=470.49,189.60");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor,p=454.19,279.67");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor,p=295.74,294.64");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor,p=6.24,171.82");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor,p=73.94,159.69");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor,p=457.63,115.55");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor,p=16.36,675.10");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor,p=79.98,677.11");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor,p=46.29,730.12");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor,p=76.17,803.08");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor,p=14.70,786.42");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor,p=121.64,860.27");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor,p=41.97,861.20");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor,p=243.29,909.45");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor,p=293.43,860.87");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor,p=294.84,942.22");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor,p=273.32,1008.78");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor,p=185.41,1016.84");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor,p=115.83,995.01");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor,p=51.37,1018.61");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor,p=200.27,953.54");
    }

}

