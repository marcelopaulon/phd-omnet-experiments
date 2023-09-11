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
        @display("i=misc/sensor,p=17.08,68.19");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=12.75,3.04");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=190.12,90.86");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=234.76,28.90");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=265.81,122.84");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=144.37,156.27");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=213.48,159.21");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=160.15,224.45");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=183.24,306.57");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=89.51,276.86");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=119.78,358.74");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=197.01,414.30");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=271.26,341.02");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=352.58,358.48");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=427.24,412.49");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=484.22,344.11");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=557.90,405.19");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=618.05,411.15");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=617.09,319.65");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=530.38,299.48");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=563.14,220.32");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=611.09,182.21");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=627.66,89.88");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=604.74,13.03");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=547.49,93.61");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=473.32,218.67");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=395.84,178.63");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=374.58,89.96");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=421.33,2.69");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=503.46,39.77");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=496.70,126.18");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=435.91,61.33");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=330.55,11.71");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=334.43,422.54");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=403.65,473.44");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=330.60,492.72");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=360.08,582.70");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=288.96,623.66");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=364.64,674.34");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=302.35,722.90");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=294.55,798.95");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=210.19,801.21");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=424.76,694.39");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=216.68,620.65");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=187.66,677.13");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=101.93,709.85");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=132.78,775.73");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=179.17,867.51");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=275.49,908.31");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor,p=176.98,994.05");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor,p=103.77,1008.02");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor,p=15.30,1008.86");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor,p=25.90,1101.76");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor,p=15.56,933.15");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor,p=102.92,938.52");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor,p=370.91,807.45");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor,p=210.11,539.36");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor,p=289.81,552.84");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor,p=153.29,568.85");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor,p=465.28,569.41");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor,p=552.23,552.86");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor,p=640.68,528.01");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor,p=711.66,526.29");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor,p=661.16,585.96");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor,p=594.62,624.61");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor,p=600.18,708.87");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor,p=489.59,680.74");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor,p=485.98,778.19");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor,p=451.56,844.29");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor,p=790.67,536.10");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor,p=873.61,517.97");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor,p=848.60,460.50");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor,p=892.41,360.71");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor,p=969.19,343.40");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor,p=1050.67,292.12");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor,p=1073.52,363.24");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor,p=795.94,392.03");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor,p=828.68,249.32");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor,p=813.83,176.90");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor,p=889.62,241.65");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor,p=760.43,328.97");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor,p=1119.25,211.67");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor,p=1118.96,143.54");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor,p=1202.78,144.52");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor,p=1243.92,82.93");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor,p=1283.92,151.83");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor,p=1338.23,222.56");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor,p=1383.94,174.63");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor,p=1379.91,108.68");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor,p=1322.10,39.61");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor,p=1381.11,26.25");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor,p=1448.78,143.88");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor,p=1446.16,210.96");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor,p=1313.72,294.23");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor,p=1227.03,254.93");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor,p=1170.70,303.80");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor,p=1024.94,204.75");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor,p=940.70,475.84");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor,p=996.77,429.87");
    }

}

