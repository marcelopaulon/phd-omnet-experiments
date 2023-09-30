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

network DADCA_ACK_n100_u16_a8000m2_2
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
        @display("i=misc/sensor;p=634.59,7885.16");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=970.65,7775.97");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1689.95,7818.12");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2143.70,7914.74");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3019.73,7783.32");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4685.78,7799.40");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5297.32,7786.45");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5856.04,7842.49");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=255.28,7519.61");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3602.58,7604.24");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4107.76,7525.94");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=719.98,7312.84");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1424.90,7326.30");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2326.88,7302.54");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2786.33,7481.66");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3273.93,7453.62");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5056.43,7293.66");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5659.15,7352.59");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=122.47,7147.77");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1072.30,7149.19");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=3524.97,7142.44");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=4348.90,7013.65");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5905.65,7104.04");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=509.90,6960.08");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=1179.18,6767.27");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1751.28,6995.92");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2318.54,6948.77");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3002.21,6914.98");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3960.31,6946.92");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5376.34,6769.99");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=325.69,6626.58");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1639.79,6554.69");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2078.79,6647.71");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2592.48,6602.25");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3393.38,6662.17");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4024.12,6537.17");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4740.05,6732.22");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5812.87,6523.85");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=497.94,6292.85");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=893.18,6355.11");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4371.05,6334.61");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=4935.44,6326.30");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1205.40,6120.84");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=1604.00,6166.78");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2294.89,6097.90");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2907.04,6205.00");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4633.73,6082.59");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=5570.09,6035.69");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=219.31,5970.48");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=880.46,5834.52");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=1948.38,5964.82");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=2663.50,5860.92");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=3229.48,5967.11");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=4057.10,5930.81");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=5097.76,5931.15");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=1349.32,5710.19");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=3637.60,5662.98");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=4500.63,5754.37");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=1235.25,5320.70");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=2114.70,5397.29");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=2523.51,5494.52");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=3087.25,5279.40");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=5090.62,5504.07");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=5657.31,5383.77");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=633.85,5150.67");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=1620.14,5160.78");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=3929.25,5231.23");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=4669.42,5213.48");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=5799.56,5031.23");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=1031.13,4894.76");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=2115.34,4959.61");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=2508.60,4811.79");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=5019.03,4959.39");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=5372.53,4798.71");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=229.19,4782.51");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=757.55,4574.18");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=1561.97,4633.99");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=3067.19,4664.07");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=3634.31,4779.27");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=4377.62,4705.93");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=4738.90,4709.76");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=1932.32,4451.03");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=2656.82,4355.57");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=3166.83,4322.60");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=4164.15,4375.28");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=4840.69,4346.27");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=5975.85,4442.05");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=137.48,4093.44");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=1211.30,4238.58");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=2308.87,4268.38");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=2909.48,4083.57");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=553.10,4033.84");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=1130.79,3889.11");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=1920.69,3910.51");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=3646.73,3953.64");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=5667.73,3927.71");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=108.32,3706.52");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=645.22,3547.36");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=1511.56,3772.61");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=2536.70,3791.88");
    }

}

