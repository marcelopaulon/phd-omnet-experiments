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

network DADCA_ACK_n100_a8000m2_0
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
        @display("i=misc/sensor;p=883.69,7964.93");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1724.41,7820.51");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2366.67,7871.77");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2996.95,7786.65");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3571.91,7760.89");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4606.95,7926.54");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=204.45,7575.79");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=624.71,7714.91");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1174.22,7667.78");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1497.55,7529.12");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4098.28,7667.65");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4460.01,7575.68");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5114.87,7567.24");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5462.12,7663.69");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2217.50,7417.12");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2883.55,7430.10");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4002.94,7326.95");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4806.24,7401.15");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5984.64,7264.58");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=683.47,7213.33");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1181.09,7225.59");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3565.96,7153.92");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4466.70,7203.93");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5276.25,7185.10");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=137.33,6856.66");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=904.44,6868.81");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2165.97,6804.08");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2682.71,6768.69");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3683.54,6819.23");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4076.85,6980.83");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4981.63,6924.89");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=5630.88,6976.69");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=5973.13,6830.19");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=533.03,6636.33");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1622.59,6708.82");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3255.88,6732.29");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4280.87,6656.93");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4821.84,6564.08");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5184.99,6571.23");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=302.60,6274.30");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=790.00,6274.59");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1243.68,6450.74");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2499.17,6280.64");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2943.79,6512.24");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3938.75,6304.80");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5800.96,6304.16");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2135.22,6238.77");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3400.42,6189.65");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4950.00,6050.36");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5413.98,6115.83");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=508.73,5838.46");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=1203.01,5887.50");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=2761.52,5789.51");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=3143.15,5781.49");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=3690.83,5931.95");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=5681.56,5850.44");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=824.53,5560.21");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=1310.16,5549.43");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=1732.70,5735.36");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=2173.89,5718.62");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=3998.92,5629.90");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=4534.17,5666.24");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=5326.36,5717.36");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=5983.02,5604.00");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=61.72,5386.25");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=464.34,5298.93");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=2434.44,5332.25");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=3400.30,5499.32");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=5086.65,5392.64");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=5688.75,5361.34");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=854.21,5083.88");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=1268.39,5160.57");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=1828.61,5063.15");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=2845.66,5199.96");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=4542.08,5265.99");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=97.30,4918.05");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=2944.30,4840.94");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=3393.16,4832.91");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=3852.72,4852.37");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=4458.01,4913.20");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=5070.57,4909.75");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=5642.81,4993.12");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=415.57,4724.22");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=777.26,4633.36");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=1384.52,4582.63");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=2043.75,4759.35");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=2489.48,4659.79");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=93.33,4456.23");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=1779.91,4413.15");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=2982.47,4411.08");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=3646.44,4372.49");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=4236.88,4289.22");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=5001.31,4397.79");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=5465.93,4355.46");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=5848.65,4452.20");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=262.96,4073.29");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=796.39,4149.36");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=2264.68,4271.78");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=2714.30,4147.22");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=3217.14,4081.71");
    }

}

