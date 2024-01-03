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

network DADCA_ACK_n100_u3_a8000m2_0
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
        @display("i=misc/sensor;p=282.36,7816.80");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=755.23,7764.46");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1419.56,7907.56");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2659.72,7917.92");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3298.60,7760.91");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3868.93,7922.85");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5530.44,7984.82");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4767.84,7721.52");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5190.00,7524.99");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=473.90,7479.02");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1129.22,7279.94");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1699.25,7502.18");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2210.17,7476.57");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2863.87,7485.72");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3330.12,7318.10");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3912.20,7286.16");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5678.72,7343.83");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=207.72,7187.56");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=760.35,7083.74");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1738.56,7142.80");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2106.66,7091.62");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2928.59,7140.78");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4420.73,7226.24");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4969.18,7225.95");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2519.57,6981.44");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=3595.41,6940.14");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=4448.55,6828.74");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4952.13,6873.86");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5349.65,6920.87");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=6.05,6582.52");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=423.90,6574.40");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1269.83,6631.31");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2366.34,6597.54");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3307.33,6710.57");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4062.58,6630.71");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4745.63,6571.63");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5329.34,6521.07");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5672.19,6696.30");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1993.44,6458.02");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2693.83,6418.98");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3359.35,6299.54");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3795.56,6325.16");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4397.19,6423.83");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=178.93,6039.47");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=493.14,6195.51");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1088.09,6117.66");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1591.07,6191.77");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=2916.69,6109.90");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4732.84,6092.68");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5537.81,6108.20");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=697.21,5811.69");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=1527.59,5824.16");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=1905.41,5899.31");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=2286.86,5890.83");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=4303.49,5974.47");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=5111.72,5948.04");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=141.56,5548.33");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=3504.64,5703.36");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=4001.49,5630.56");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=4459.53,5626.49");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=4964.98,5620.35");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=5702.14,5576.73");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=523.93,5505.82");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=1036.84,5521.90");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=2072.07,5492.84");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=3034.46,5284.59");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=3700.33,5305.03");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=4638.00,5311.70");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=880.56,5109.69");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=1425.65,5257.76");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=2543.56,5247.26");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=4300.52,5127.20");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=5270.99,5201.27");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=5896.77,5092.91");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=1853.85,5009.44");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=2242.84,4816.22");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=3325.53,4953.95");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=4732.71,4859.34");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=372.36,4537.34");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=873.68,4550.43");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=1281.16,4766.00");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=2853.20,4710.00");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=3866.52,4739.75");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=4223.77,4722.46");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=5090.94,4736.03");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=5774.68,4586.92");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=1534.99,4502.72");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=2051.06,4485.38");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=2688.25,4326.92");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=3164.11,4476.64");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=3861.41,4299.00");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=5407.01,4479.79");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=399.76,4064.66");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=1821.05,4075.93");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=4383.31,4082.27");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=5091.82,4259.59");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=5517.49,4103.16");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=998.47,3991.07");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=1548.55,3846.70");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=2283.97,3882.91");
    }

}

