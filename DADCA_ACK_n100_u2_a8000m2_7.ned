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

network DADCA_ACK_n100_u2_a8000m2_7
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
        @display("i=misc/sensor;p=133.24,7948.83");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=757.86,7958.07");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2484.49,7856.69");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3066.42,7756.03");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5915.52,7936.40");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=188.07,7552.92");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=998.10,7552.64");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1399.85,7559.25");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3717.81,7737.82");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5758.48,7586.69");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1873.58,7368.06");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2304.70,7356.67");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4186.58,7414.50");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5213.99,7426.89");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=134.33,7174.61");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=683.84,7210.54");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1257.17,7028.93");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3138.36,7206.13");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3781.90,7047.33");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4375.14,7100.21");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=4795.62,7202.39");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=5986.61,7140.88");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1740.95,6802.63");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=2094.82,6764.90");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2616.38,6949.43");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5119.31,6903.51");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=5600.27,6992.51");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=682.47,6735.51");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2960.97,6694.59");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3663.84,6711.72");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4331.07,6612.95");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4811.34,6589.45");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=5397.66,6559.33");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2344.44,6330.73");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2680.69,6441.14");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3981.74,6363.61");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=286.07,6242.66");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=817.28,6134.25");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1486.84,6209.27");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2109.47,6066.05");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3014.79,6158.81");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3476.63,6181.46");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4608.57,6183.18");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5141.68,6216.05");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5814.48,6158.19");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=342.80,5819.51");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1692.96,5822.11");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3344.93,5809.66");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3977.99,5898.89");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4879.30,5864.86");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=1075.57,5717.74");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=2327.36,5549.25");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=2747.19,5578.47");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=4117.92,5574.46");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=4487.00,5752.26");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=5302.94,5689.69");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=55.04,5333.23");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=408.85,5471.38");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=746.13,5337.00");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=1873.33,5421.98");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=4954.55,5312.86");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=5638.71,5459.54");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=1088.96,5217.13");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=2549.38,5146.46");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=3230.06,5138.85");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=3796.96,5237.62");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=4123.65,5105.77");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=4548.75,5104.17");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=5823.35,5064.02");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=1408.93,5014.70");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=1846.72,4932.87");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=5309.50,4987.16");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=95.63,4612.31");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=498.46,4732.38");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=1145.28,4641.46");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=3736.81,4640.64");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=4314.48,4712.93");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=4788.51,4780.22");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=5939.81,4701.96");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=895.95,4350.80");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=1647.83,4298.32");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=2179.28,4495.43");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=2687.99,4527.04");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=3272.44,4466.85");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=4622.67,4364.06");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=4994.64,4330.99");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=5391.14,4463.12");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=5909.27,4343.37");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=397.31,4064.36");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=2040.45,4060.72");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=2855.76,4164.31");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=3740.75,4134.84");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=4303.10,4075.91");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=5514.15,4058.95");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=1228.27,3981.15");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=3997.94,3864.76");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=4645.65,3917.05");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=661.66,3731.05");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=1686.94,3694.77");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=2589.89,3659.68");
    }

}

