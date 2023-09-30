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

network DADCA_ACK_n100_u4_a8000m2_8
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
        @display("i=misc/sensor;p=49.20,7884.39");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=842.38,7948.83");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2373.70,7886.58");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4069.47,7985.26");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5324.62,7755.69");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=403.38,7521.75");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1318.69,7601.43");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1766.38,7604.22");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2130.25,7556.55");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3158.48,7589.42");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4326.47,7540.45");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=845.12,7276.85");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2777.32,7330.52");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3792.44,7388.70");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4819.40,7365.41");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5227.10,7351.58");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5585.03,7356.61");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=189.52,7148.78");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1471.25,7118.58");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1926.66,7110.68");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=620.99,6877.09");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1061.55,6777.79");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=2467.73,6936.54");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=2951.63,6998.32");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3432.87,6988.64");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=4485.93,6921.49");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=4847.85,6957.34");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5329.60,7008.73");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5916.23,6775.83");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=160.76,6661.33");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=537.80,6532.39");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1601.41,6594.27");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2196.24,6660.67");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3032.39,6559.38");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4047.50,6744.72");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3697.44,6274.71");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4938.94,6462.72");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=807.41,6262.14");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1337.60,6148.24");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1765.61,6171.21");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2461.18,6227.45");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3290.11,6207.91");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4526.93,6222.81");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=165.25,5942.20");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=521.10,5896.45");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=958.84,5814.67");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1977.76,5807.52");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=2409.30,5826.43");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3419.70,5788.04");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=3768.28,5912.49");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=4632.24,5789.53");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=5104.45,5916.76");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=140.69,5547.08");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=1512.40,5610.77");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=2769.03,5769.50");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=3934.86,5539.93");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=5456.76,5598.63");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=2561.04,5478.74");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=2998.62,5287.13");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=3569.03,5340.78");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=4412.68,5490.93");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=4883.61,5464.14");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=5746.50,5297.97");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=322.17,5181.04");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=800.40,5106.66");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=1469.38,5177.54");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=2129.67,5262.55");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=2538.78,5032.29");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=3337.05,5040.11");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=4011.55,5180.98");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=5226.12,5274.16");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=1908.53,4846.17");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=3048.27,4792.53");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=4638.15,4988.17");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=5437.39,4791.47");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=596.33,4745.88");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=1270.03,4765.24");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=2212.16,4645.10");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=2708.26,4673.18");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=3464.28,4681.80");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=3811.51,4574.48");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=5009.07,4667.08");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=5753.74,4537.66");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=1945.58,4292.81");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=3154.09,4358.73");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=4405.55,4532.46");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=5130.25,4320.96");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=151.91,4185.09");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=620.79,4149.38");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=980.12,4045.87");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=1420.76,4222.65");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=2453.88,4244.85");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=4636.99,4094.08");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=5476.16,4197.96");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=1266.39,3843.99");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=1731.16,3936.21");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=2486.57,3889.87");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=2978.98,3870.64");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=3528.24,3985.63");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=4003.43,4030.99");
    }

}

