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
        @display("i=misc/sensor,p=47.28,57.56");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=124.28,13.56");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=6.36,120.39");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=190.75,66.74");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=264.32,4.86");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=300.08,94.96");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=336.40,22.08");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=408.62,40.67");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=403.06,111.06");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=469.26,52.73");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=548.89,50.50");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=583.37,107.62");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=494.18,152.18");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=500.43,0.63");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=187.76,221.64");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=174.11,135.66");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=231.95,166.17");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=203.28,298.70");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=142.40,299.66");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=279.17,350.77");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=292.14,287.27");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=348.83,333.22");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=373.21,242.44");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=456.14,257.41");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=513.89,289.48");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=446.07,333.40");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=274.89,423.31");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=359.18,430.16");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=401.41,474.86");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=109.70,178.21");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=620.55,275.48");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=564.39,253.76");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=587.88,195.37");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=685.19,209.84");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=746.76,149.14");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=804.31,202.43");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=817.57,60.80");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=751.81,34.03");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=672.33,86.98");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=500.96,216.58");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=7.19,204.34");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=309.54,216.25");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=510.26,397.82");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=582.04,483.13");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=538.76,539.11");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=477.96,578.76");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=388.12,611.69");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=499.41,642.98");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=567.22,697.34");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor,p=581.58,617.47");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor,p=642.68,634.94");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor,p=678.69,691.61");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor,p=733.94,740.94");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor,p=694.18,547.45");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor,p=745.94,469.83");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor,p=711.34,392.09");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor,p=590.94,419.49");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor,p=198.27,417.62");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor,p=139.62,390.20");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor,p=79.82,406.83");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor,p=99.07,497.49");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor,p=5.47,464.86");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor,p=58.31,548.03");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor,p=111.56,619.77");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor,p=13.32,619.00");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor,p=141.39,563.18");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor,p=195.60,625.54");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor,p=140.98,696.62");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor,p=61.60,684.03");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor,p=75.31,761.25");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor,p=125.18,842.37");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor,p=209.51,817.94");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor,p=239.17,762.08");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor,p=280.50,830.31");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor,p=279.18,698.17");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor,p=351.87,740.61");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor,p=392.02,683.18");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor,p=319.61,646.31");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor,p=255.30,522.51");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor,p=320.96,537.77");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor,p=448.74,715.32");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor,p=382.33,816.01");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor,p=498.45,827.03");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor,p=266.19,938.88");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor,p=170.42,951.65");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor,p=271.54,1030.66");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor,p=194.74,1008.10");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor,p=216.99,888.08");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor,p=80.23,963.97");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor,p=40.50,892.79");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor,p=38.11,815.26");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor,p=604.21,769.81");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor,p=679.61,805.33");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor,p=522.48,904.79");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor,p=438.76,983.87");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor,p=362.48,979.84");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor,p=342.81,921.88");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor,p=163.06,753.66");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor,p=15.57,723.89");
    }

}

