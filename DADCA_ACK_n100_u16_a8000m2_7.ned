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

network DADCA_ACK_n100_u16_a8000m2_7
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
        @display("i=misc/sensor;p=500.70,7753.15");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1486.23,7894.43");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2702.29,7887.11");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3849.08,7875.68");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4340.24,7802.81");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4863.28,7892.59");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5256.87,7855.32");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1100.65,7733.29");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2014.70,7610.36");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3298.28,7651.09");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5050.60,7550.94");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5685.36,7698.70");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=95.59,7349.13");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1281.44,7275.09");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2307.72,7326.47");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3084.13,7272.23");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3873.63,7416.32");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4459.73,7448.98");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5381.35,7389.60");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=619.37,7230.46");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1953.76,7057.25");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2582.75,7035.09");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4228.54,7015.21");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4646.42,7089.34");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5823.34,7245.12");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2173.52,6771.14");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3306.44,6941.61");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3665.37,6931.58");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5781.84,6836.22");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=675.10,6645.62");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1795.55,6611.84");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2810.30,6590.34");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4044.36,6595.58");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=4830.55,6635.57");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5435.43,6755.10");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=285.53,6339.11");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=1149.08,6428.30");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1648.84,6272.99");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4543.35,6404.45");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5566.59,6335.08");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5950.02,6445.89");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=823.64,6203.20");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2322.48,6184.50");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3178.61,6069.32");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3576.94,6259.33");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=4187.14,6190.93");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4578.70,6030.99");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=5244.01,6121.66");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=116.26,5902.63");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=656.05,5858.08");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=1113.92,5933.11");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=2036.95,5956.17");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=2822.99,5911.98");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=3743.26,5821.74");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=5957.59,5953.77");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=451.83,5567.90");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=1499.32,5723.41");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=1862.40,5555.63");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=3378.84,5730.81");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=4288.97,5717.78");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=5043.58,5537.70");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=5612.16,5679.56");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=2.65,5375.76");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=870.53,5394.44");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=1353.93,5397.93");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=2334.03,5462.94");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=2857.75,5438.66");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=3937.36,5489.27");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=4589.32,5447.20");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=5907.95,5425.63");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=352.03,5156.42");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=1672.47,5104.94");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=3052.15,5133.38");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=3684.12,5209.61");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=4226.79,5180.65");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=689.41,4857.64");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=1337.41,4963.33");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=2183.57,4994.02");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=3386.38,5017.12");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=4679.64,4996.04");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=5076.96,4899.76");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=5443.23,5029.73");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=5852.95,4790.20");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=127.09,4603.95");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=1221.59,4573.49");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=1768.99,4678.79");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=2787.90,4770.99");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=3601.16,4676.13");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=3949.25,4780.78");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=4433.79,4715.80");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=5119.55,4544.39");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=945.53,4346.51");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=1592.00,4339.93");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=2440.43,4436.51");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=3176.09,4513.51");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=4197.90,4365.28");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=4725.17,4381.99");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=5470.62,4385.92");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=143.19,4212.48");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=4918.27,4084.95");
    }

}

