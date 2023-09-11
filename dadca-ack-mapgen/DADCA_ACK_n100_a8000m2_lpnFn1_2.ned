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
        @display("i=misc/sensor,p=24.56,121.65");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=167.31,213.87");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=96.13,190.71");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=145.51,29.24");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=219.57,69.73");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=183.04,135.56");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=159.24,330.66");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=103.65,359.97");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=91.28,422.85");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=139.42,461.70");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=182.56,509.81");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=171.69,588.45");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=95.40,622.91");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=99.19,549.07");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=244.01,553.60");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=324.18,530.13");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=277.06,488.09");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=323.13,409.49");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=261.34,400.61");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=233.44,337.06");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=290.80,307.66");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=389.01,344.94");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=351.43,279.67");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=313.94,218.39");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=402.40,176.11");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=453.61,215.14");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=463.20,381.91");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=544.26,333.52");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=532.33,271.92");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=444.17,303.74");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=414.55,56.77");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=480.85,38.34");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=563.84,28.75");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=585.19,118.64");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=621.37,173.82");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=629.19,245.26");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=505.47,144.15");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=677.25,129.50");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=745.43,131.99");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=742.90,41.28");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=766.42,208.32");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=850.16,176.63");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=935.08,156.43");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=875.34,249.50");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=890.77,111.49");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=836.14,46.44");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=919.55,19.47");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=804.88,103.35");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=1010.81,69.27");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor,p=1037.58,132.68");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor,p=1075.71,61.51");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor,p=1157.04,3.10");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor,p=1164.43,82.05");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor,p=1116.79,144.50");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor,p=1174.53,219.48");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor,p=1089.27,230.63");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor,p=1029.97,275.48");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor,p=1001.66,182.67");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor,p=1188.25,291.88");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor,p=1222.99,357.76");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor,p=1264.47,308.96");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor,p=1177.35,419.37");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor,p=1238.21,419.00");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor,p=1095.30,317.74");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor,p=1038.94,353.34");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor,p=953.48,243.33");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor,p=639.60,30.65");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor,p=912.72,342.41");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor,p=946.21,438.87");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor,p=972.74,372.76");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor,p=836.76,400.06");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor,p=954.46,545.97");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor,p=852.12,487.20");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor,p=780.89,445.58");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor,p=691.46,412.32");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor,p=634.49,444.83");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor,p=658.19,517.20");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor,p=664.03,598.05");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor,p=742.04,619.94");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor,p=698.62,704.09");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor,p=760.36,765.13");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor,p=715.19,828.96");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor,p=660.35,760.78");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor,p=597.04,786.26");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor,p=545.13,840.02");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor,p=546.55,749.42");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor,p=512.96,681.00");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor,p=610.67,675.89");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor,p=829.06,709.81");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor,p=862.04,797.80");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor,p=937.94,799.18");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor,p=954.88,705.33");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor,p=1005.90,668.03");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor,p=1056.64,707.03");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor,p=1062.43,623.63");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor,p=1000.77,602.36");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor,p=1009.74,791.79");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor,p=888.00,881.18");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor,p=823.72,986.33");
    }

}

