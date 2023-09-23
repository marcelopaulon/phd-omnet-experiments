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

network DADCA_ACK_n100_u2_a8000m2_9
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
        @display("i=misc/sensor;p=793.72,7761.09");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1224.73,7883.79");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3856.88,7757.57");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4615.09,7948.81");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=277.64,7684.60");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=1099.11,7551.14");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1742.05,7567.92");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2199.62,7538.72");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2786.92,7575.35");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3301.42,7630.89");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4910.31,7710.74");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=767.62,7338.98");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1470.44,7346.87");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4226.05,7482.84");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4744.04,7313.09");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5124.27,7373.28");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5666.97,7272.95");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=316.37,7108.03");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1112.58,7159.47");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1969.04,7094.29");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2423.19,7235.03");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2853.85,7205.70");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3430.81,7217.36");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3837.90,7190.65");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=933.87,6775.80");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1645.28,6866.11");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2503.41,6853.47");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3244.82,6919.86");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4285.39,6946.49");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4635.48,6881.49");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5188.73,6953.56");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=5948.91,6956.58");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=189.92,6547.27");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=562.61,6634.60");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1304.51,6683.83");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2993.76,6535.83");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3634.30,6686.88");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5646.26,6612.06");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=957.41,6413.11");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1950.02,6443.54");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4162.08,6494.83");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=4999.40,6366.32");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=207.50,6145.54");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=1332.84,6248.36");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1986.47,6093.21");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2434.32,6225.07");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4491.13,6151.37");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=5343.19,6235.60");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5710.45,6077.36");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=615.30,5850.92");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=1173.52,5824.04");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=1548.69,5784.34");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=2577.48,5781.96");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=3056.52,5990.08");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=3479.63,5941.05");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=3842.99,6018.45");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=4170.07,5826.20");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=4658.57,5814.44");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=77.88,5527.06");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=2152.33,5630.88");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=3049.00,5609.10");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=3696.39,5613.18");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=5236.89,5622.41");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=5864.27,5747.00");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=1363.75,5323.22");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=1895.34,5283.23");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=4358.41,5389.13");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=5688.67,5427.79");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=651.83,5155.44");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=1011.58,5100.26");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=2648.00,5195.71");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=3944.32,5199.86");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=4720.25,5168.96");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=382.41,4903.88");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=1904.56,4869.18");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=2231.24,5021.21");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=4172.31,4855.25");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=4927.68,4851.44");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=5300.13,4938.09");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=74.64,4647.66");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=733.29,4674.81");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=1218.73,4767.31");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=1675.63,4594.85");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=2697.81,4724.89");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=3352.62,4599.67");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=3892.22,4640.86");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=4443.84,4558.65");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=1994.31,4380.03");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=2853.10,4363.88");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=340.01,4232.42");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=795.77,4203.90");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=1147.28,4195.26");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=1552.29,4255.78");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=2504.72,4043.30");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=3517.73,4266.10");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=4281.16,4228.25");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=4886.60,4228.71");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=793.81,3807.03");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=1898.10,3959.10");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=2832.11,3803.22");
    }

}

