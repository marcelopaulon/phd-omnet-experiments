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

network DADCA_ACK_n100_u4_a8000m2_5
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
        @display("i=misc/sensor;p=147.75,7812.84");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=804.40,7936.58");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1228.27,7931.44");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=1912.60,7994.13");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2312.86,7958.36");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3839.85,7894.03");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1743.20,7686.87");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2498.42,7602.67");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3579.82,7596.78");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4339.42,7690.61");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4948.57,7725.61");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5635.68,7675.14");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=517.67,7358.58");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1778.91,7263.64");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3132.47,7449.11");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4028.67,7416.22");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4589.18,7405.69");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1136.95,7252.82");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2230.34,7251.55");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2729.72,7062.11");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=3955.28,7040.27");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=4943.21,7200.29");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5774.97,7136.26");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=744.09,6841.85");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=1322.01,6781.42");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2019.59,6823.77");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3262.35,6980.96");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4734.08,6782.81");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5405.81,6809.02");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=356.39,6683.05");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1725.19,6625.16");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2941.91,6527.11");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=3744.90,6619.35");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=4354.55,6753.30");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5083.37,6591.28");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5758.10,6596.30");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=55.51,6502.45");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=388.81,6306.00");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=713.04,6456.55");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=3366.13,6270.33");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4141.01,6429.70");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1231.43,6171.27");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2139.79,6117.21");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2640.62,6108.22");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3042.61,6084.58");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=4629.08,6233.57");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5597.98,6193.79");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=148.41,5987.88");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=570.50,5931.14");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1538.30,5923.12");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=2482.20,5785.74");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=3327.70,5850.12");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=3676.02,5801.80");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=4341.97,5788.95");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=5159.89,5845.40");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=5812.01,5912.15");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=239.60,5641.92");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=886.62,5536.94");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=1488.35,5554.08");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=2004.79,5740.97");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=2924.00,5555.11");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=5631.02,5545.79");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=3521.93,5299.21");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=3906.68,5381.62");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=4681.64,5467.34");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=168.50,5187.27");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=651.48,5265.18");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=1340.24,5156.31");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=1792.33,5188.70");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=2609.15,5159.23");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=5247.22,5063.61");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=581.87,4901.56");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=1654.12,4859.08");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=2142.72,4917.75");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=2930.50,5018.16");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=3336.43,4979.74");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=4207.16,5024.69");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=4663.37,4840.71");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=5693.91,4790.07");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=245.06,4622.29");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=997.70,4752.22");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=1837.98,4557.08");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=2518.88,4675.98");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=2851.80,4547.63");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=3322.05,4535.83");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=3654.57,4686.37");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=4265.98,4650.52");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=5202.43,4539.42");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=659.24,4371.41");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=1328.22,4376.79");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=4614.98,4310.04");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=5499.10,4291.01");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=9.98,4170.40");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=1917.50,4090.77");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=2377.20,4135.06");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=2765.86,4142.72");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=3853.10,4106.46");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=5951.12,4284.75");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=363.37,3958.99");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=846.42,3981.25");
    }

}

