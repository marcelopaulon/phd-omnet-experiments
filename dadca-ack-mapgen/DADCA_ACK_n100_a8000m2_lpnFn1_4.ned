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

network DADCA_ACK_n100_a8000m2_lpnFn1_4
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
        @display("i=misc/sensor;p=100.00,100.00");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=96.26,33.56");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=76.81,199.29");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=176.37,194.91");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=209.54,137.79");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=277.62,132.20");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=32.29,147.48");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=27.85,77.41");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=235.10,254.52");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=313.74,258.20");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=317.84,323.63");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=206.60,355.09");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=37.37,259.58");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=139.35,270.14");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=75.92,306.23");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=416.79,218.84");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=407.35,279.64");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=492.63,265.26");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=551.08,221.47");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=564.83,290.50");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=503.39,347.89");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=421.46,371.80");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=229.40,416.16");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=217.10,502.39");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=129.90,475.72");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=104.12,405.46");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=24.74,466.01");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=13.09,554.97");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=65.22,620.64");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=88.75,563.07");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=111.50,699.00");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=210.38,700.11");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=257.72,623.46");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=208.27,575.76");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=154.00,648.42");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=205.00,809.42");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=147.43,791.88");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=67.67,813.02");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=150.34,854.84");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=99.75,903.30");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=222.60,973.73");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=276.45,1018.45");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=316.60,1070.02");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=388.04,1042.32");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=443.64,1109.96");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=474.16,1024.83");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=191.30,1104.30");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=195.92,1176.89");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=270.30,915.95");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=299.84,844.90");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=359.19,863.16");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=430.74,845.06");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=393.05,761.03");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=506.17,836.39");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=455.54,790.36");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=520.33,704.82");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=607.96,641.00");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=571.85,835.53");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=622.88,784.98");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=686.14,798.89");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=660.95,861.25");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=777.55,838.57");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=865.44,817.26");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=960.50,825.34");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=1022.35,786.30");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=1107.95,796.74");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=1095.90,720.34");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=1169.09,680.01");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=1076.59,644.98");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=1087.87,550.86");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=1017.46,541.70");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=1054.41,490.17");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=924.18,438.32");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=903.86,513.76");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=854.03,572.38");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=793.31,467.21");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=803.61,404.55");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=736.17,441.51");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=749.25,609.98");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=899.83,676.36");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=955.57,644.67");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=803.83,676.84");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=723.53,546.30");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=902.46,359.15");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=858.72,317.85");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=758.32,345.69");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=675.67,379.36");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=619.14,225.61");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=712.50,255.66");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=820.84,226.05");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=715.58,146.99");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=811.26,154.92");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=882.18,155.50");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=859.75,89.23");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=796.91,296.84");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=582.39,437.76");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=627.78,581.95");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=671.88,665.13");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=581.89,541.07");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=643.98,722.20");
    }

}

