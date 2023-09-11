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

network DADCA_ACK_n100_a8000m2_lpnFn1_3
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
        @display("i=misc/sensor,p=83.48,164.63");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=6.91,184.20");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=81.52,244.30");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=149.79,237.27");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=163.64,307.33");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=256.94,286.46");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=290.24,193.79");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=207.16,181.27");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=334.88,236.26");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=19.58,267.19");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=16.16,104.01");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=81.46,42.77");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=180.67,105.71");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=293.02,110.81");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=388.32,171.94");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=373.92,106.26");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=398.89,259.55");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=495.27,244.68");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=461.55,158.70");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=446.43,88.83");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=274.74,43.95");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=343.50,32.49");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=505.06,61.72");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=571.90,104.68");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=465.10,4.49");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=371.37,318.85");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=528.39,157.12");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=565.89,36.92");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=631.14,150.17");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=692.20,128.33");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=764.16,71.13");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=856.01,78.35");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=819.16,130.81");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=900.93,129.06");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=965.29,159.29");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=1041.31,181.72");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=942.12,252.94");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=876.24,266.04");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=915.93,352.05");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=829.77,370.80");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=762.90,423.55");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=686.81,441.74");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=609.04,454.69");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=538.20,488.83");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=470.76,415.74");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=493.93,359.60");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=306.41,333.54");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=359.10,438.47");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=459.35,309.99");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor,p=415.28,364.27");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor,p=209.50,400.67");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor,p=124.91,392.03");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor,p=90.76,330.11");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor,p=195.38,2.78");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor,p=2.86,45.48");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor,p=413.01,516.43");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor,p=350.26,543.88");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor,p=301.49,483.57");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor,p=239.87,602.52");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor,p=297.99,620.65");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor,p=363.81,687.78");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor,p=417.31,623.42");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor,p=467.21,576.00");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor,p=470.23,490.21");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor,p=591.03,604.76");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor,p=476.00,679.87");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor,p=416.02,735.25");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor,p=413.93,797.08");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor,p=481.54,794.36");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor,p=501.92,876.99");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor,p=570.93,812.79");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor,p=576.44,673.63");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor,p=640.31,666.38");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor,p=711.43,648.16");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor,p=762.60,599.37");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor,p=739.56,522.90");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor,p=674.85,554.21");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor,p=831.79,516.79");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor,p=708.17,368.08");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor,p=638.46,341.99");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor,p=617.35,535.76");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor,p=171.60,662.89");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor,p=116.33,732.10");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor,p=169.10,762.43");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor,p=239.49,773.37");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor,p=42.99,710.47");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor,p=78.73,800.62");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor,p=176.59,886.93");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor,p=159.22,958.79");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor,p=113.07,889.44");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor,p=64.96,976.59");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor,p=64.66,1047.68");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor,p=43.16,1140.50");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor,p=90.20,1192.43");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor,p=6.16,1204.35");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor,p=146.74,1147.87");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor,p=162.33,1210.94");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor,p=251.91,1208.39");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor,p=217.92,1121.17");
    }

}

