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

network DADCA_ACK_n100_u2_a8000m2_0
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
        @display("i=misc/sensor;p=176.79,7830.34");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1036.05,7899.88");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2161.39,7924.31");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2605.23,7962.99");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3721.32,7873.78");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5599.09,7806.93");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=731.64,7533.02");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1579.72,7622.12");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2018.51,7591.31");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2466.79,7602.06");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3355.67,7686.39");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3691.15,7509.59");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4055.95,7731.40");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4855.68,7596.65");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5178.00,7747.70");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1163.07,7475.83");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4440.52,7281.04");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5467.28,7260.10");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=535.37,7204.98");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1222.23,7127.28");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2653.09,7044.23");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3090.36,7128.62");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3707.43,7053.35");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5127.62,7138.45");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5981.74,7245.85");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=105.48,6902.06");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=656.18,6818.16");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2251.25,6944.61");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4221.34,6930.04");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4754.08,6909.85");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=992.90,6614.88");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2101.70,6622.58");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2700.17,6661.45");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3395.93,6565.30");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5672.73,6701.41");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=94.43,6442.30");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=487.26,6463.84");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1614.31,6449.21");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=2436.29,6402.95");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2885.85,6355.53");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3778.97,6485.60");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=4459.38,6513.37");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5292.36,6357.84");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5933.51,6424.15");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=861.50,6204.50");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2062.27,6221.96");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3614.37,6136.80");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1422.05,5838.64");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=2349.84,5842.78");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4048.83,6008.25");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=4519.05,5871.05");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=4967.91,5873.34");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=5738.87,5926.77");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=405.20,5655.67");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=1929.57,5713.41");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=2982.39,5666.15");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=3941.87,5665.81");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=5119.15,5526.59");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=5458.40,5682.56");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=5933.35,5579.49");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=670.39,5407.02");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=1144.21,5492.47");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=2210.23,5373.86");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=3383.35,5500.16");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=4757.51,5488.41");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=5462.96,5322.06");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=312.99,5135.46");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=1415.78,5073.98");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=1720.68,5263.98");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=2405.91,5062.21");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=2847.71,5198.22");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=3495.82,5139.38");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=4003.21,5241.18");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=670.01,5001.96");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=1108.56,4894.32");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=1999.89,4934.03");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=3291.61,4849.61");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=4629.35,5029.74");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=4980.58,4831.79");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=5722.78,5009.73");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=442.12,4712.19");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=1605.51,4687.74");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=2319.77,4586.16");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=2763.81,4714.44");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=4066.44,4778.77");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=4467.52,4686.03");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=5502.58,4638.14");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=330.53,4364.85");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=847.12,4500.57");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=3248.01,4347.22");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=3847.78,4400.92");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=4665.45,4296.87");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=5019.20,4302.35");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=5842.04,4302.47");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=1875.40,4282.77");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=2353.20,4210.16");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=2741.37,4280.31");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=4207.17,4221.60");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=5422.09,4270.53");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=1137.99,3912.62");
    }

}

