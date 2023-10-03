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

network DADCA_ACK_n100_u8_a8000m2_7
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
        @display("i=misc/sensor;p=3346.11,7869.30");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=4325.44,7806.78");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=4771.43,7981.42");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=5370.45,7939.03");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2674.31,7684.62");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3039.38,7546.37");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3703.44,7721.75");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4874.03,7516.48");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5787.84,7720.56");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2263.05,7276.35");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3394.11,7372.59");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4103.99,7324.45");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4515.55,7360.56");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5412.99,7487.60");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1350.41,7198.06");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2950.42,7102.25");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4610.80,7015.73");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5165.08,7088.59");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5810.16,7257.44");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=464.27,6953.90");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=917.86,6832.26");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1743.71,6978.19");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=2600.62,6976.75");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3541.80,6974.34");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4261.67,6917.60");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5806.13,6892.08");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1228.41,6586.95");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2865.00,6608.19");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3940.73,6681.82");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4654.52,6645.86");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5383.34,6652.35");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=317.71,6425.93");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=719.73,6463.76");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1990.88,6406.73");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2368.33,6450.21");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3151.99,6371.06");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=625.17,6072.69");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1347.30,6247.66");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1760.53,6069.75");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=3070.61,6023.93");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3750.69,6200.15");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=4225.32,6170.44");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5226.30,6244.38");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5777.19,6198.62");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=222.54,5881.21");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=909.44,5836.70");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2607.54,6014.68");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4125.48,5785.37");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4605.47,6003.63");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5170.56,5893.72");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=5571.10,5802.78");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=522.14,5666.17");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=1464.91,5658.01");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=2085.67,5725.93");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=2652.91,5568.55");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=3601.86,5624.89");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=4524.92,5591.08");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=956.29,5478.35");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=1903.81,5385.60");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=2271.70,5314.42");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=3079.83,5503.44");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=3917.52,5332.30");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=4880.10,5397.41");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=135.09,5251.09");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=724.35,5113.36");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=1484.78,5127.42");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=2793.77,5188.35");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=3358.89,5117.36");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=4202.81,5051.47");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=4550.73,5230.17");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=5146.41,5160.64");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=349.88,4912.20");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=1081.30,4826.69");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=1945.55,4836.48");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=2398.48,4928.80");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=3691.16,4963.56");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=5615.21,4859.58");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=3096.69,4538.95");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=3584.78,4612.64");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=3967.77,4692.33");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=4546.86,4565.69");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=276.34,4503.30");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=835.95,4297.96");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=1430.09,4518.25");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=1813.20,4416.35");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=2336.08,4506.70");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=5098.69,4423.25");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=5559.18,4412.10");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=275.98,4149.31");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=2773.88,4213.72");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=3196.55,4108.04");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=4230.56,4131.21");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=5914.71,4143.53");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=695.46,3903.56");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=1280.04,4030.12");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=2453.02,3998.16");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=2980.11,3831.29");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=3837.42,3931.84");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=5118.25,3993.18");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=129.03,3612.74");
    }

}

