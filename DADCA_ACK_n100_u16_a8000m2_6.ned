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

network DADCA_ACK_n100_u16_a8000m2_6
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
        @display("i=misc/sensor;p=864.35,7821.23");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1576.97,7941.41");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2004.60,7819.02");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3039.70,7856.38");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3541.46,7792.70");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4130.37,7777.66");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4582.00,7991.50");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5171.10,7755.43");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5561.77,7816.21");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5959.21,7885.98");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=326.20,7612.70");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2630.54,7728.88");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4460.77,7662.11");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1055.33,7495.62");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1420.92,7487.96");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1838.22,7429.05");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2337.40,7350.59");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3196.25,7378.83");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4329.99,7320.72");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4698.69,7383.96");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=5569.24,7448.31");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=233.45,7141.24");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=627.63,7240.50");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=1401.62,7039.27");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2793.62,7174.98");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=3717.73,7156.23");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=4638.67,7016.64");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5075.95,7116.94");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=821.00,6887.74");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2091.81,7009.44");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=3400.02,6836.13");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=3867.92,6790.87");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4267.32,6854.94");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5659.55,6829.16");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=418.99,6621.68");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=913.87,6527.76");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=1773.88,6737.36");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=2282.93,6711.88");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=2739.07,6541.28");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4832.64,6542.98");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5287.07,6519.29");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=27.50,6410.09");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1373.60,6477.18");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=1769.62,6324.90");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2444.45,6338.61");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3518.06,6345.19");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3989.39,6352.89");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4544.86,6287.54");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5683.19,6354.51");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=429.61,6240.15");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=869.41,6151.04");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=2041.72,6087.89");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=3078.83,6191.03");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=5285.70,6045.69");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=5931.37,6069.48");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=116.56,5836.28");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=1350.24,6019.51");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=2389.33,5917.66");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=2869.12,5782.15");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=3447.44,5966.49");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=3842.94,5831.87");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=4333.89,5905.16");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=4930.52,5890.45");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=993.05,5642.73");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=1386.81,5547.71");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=1783.68,5595.26");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=3187.41,5625.56");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=4490.25,5583.30");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=4869.02,5533.24");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=5243.64,5539.78");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=80.42,5295.08");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=488.95,5462.44");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=2540.90,5479.22");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=2962.51,5278.04");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=3521.77,5365.04");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=3981.77,5389.12");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=447.08,5114.42");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=1254.16,5169.09");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=2163.12,5224.37");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=5697.85,5231.37");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=1773.04,4957.69");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=2570.25,4874.55");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=3166.86,4969.32");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=3767.41,4958.23");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=4147.15,4991.13");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=4787.97,5024.81");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=5321.11,4872.21");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=5927.31,4902.52");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=358.69,4680.74");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=901.14,4701.32");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=1466.62,4764.71");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=2037.71,4642.47");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=3678.17,4588.66");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=4315.68,4572.51");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=5675.13,4596.66");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=1248.12,4332.75");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=1668.54,4347.28");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=2550.03,4490.95");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=3108.94,4333.74");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=3981.96,4387.25");
    }

}

