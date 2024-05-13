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
        @display("i=misc/sensor;p=729.93,7998.59");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1359.52,7881.48");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=4676.82,7778.38");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=5523.74,7786.38");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=0.96,7577.58");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=440.98,7536.09");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=922.75,7650.24");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3578.09,7733.65");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4119.86,7680.73");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5170.36,7678.57");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5802.76,7559.34");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1267.48,7413.24");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1822.15,7457.97");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5232.75,7261.96");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=105.14,7052.03");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=783.58,7044.19");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2251.76,7017.39");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3141.62,7230.99");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3827.21,7135.59");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4913.69,7103.45");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1747.92,6817.09");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2734.79,6777.74");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5793.57,6894.02");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=228.41,6557.85");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=1024.08,6756.83");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=3377.36,6729.86");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3691.27,6539.86");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4417.32,6625.17");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5032.72,6732.87");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=685.16,6441.02");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1313.24,6321.20");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1894.19,6295.18");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2445.71,6498.22");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2947.89,6435.18");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4072.42,6432.68");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4916.66,6400.21");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5458.20,6450.05");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5882.42,6413.49");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=112.78,6156.91");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=921.25,6180.67");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3574.95,6171.27");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=554.30,5802.97");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1592.61,5887.30");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2133.20,5989.35");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2679.96,5887.86");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3018.65,5795.05");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4114.24,5997.65");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4570.74,5976.81");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5053.82,5909.66");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5452.34,5814.32");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=5753.40,6011.62");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=19.48,5674.20");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=908.63,5560.61");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=2179.47,5617.02");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=3353.76,5666.21");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=3728.56,5628.86");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=4467.11,5558.12");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=4817.64,5532.98");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=358.64,5318.23");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=2459.08,5314.30");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=2881.46,5305.69");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=5168.29,5360.65");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=5845.16,5440.37");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=877.16,5193.49");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=1483.50,5209.88");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=3399.12,5244.51");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=4530.79,5108.19");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=160.89,4807.10");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=1780.04,4800.02");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=2118.19,4973.33");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=3103.47,5008.65");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=3458.71,4798.48");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=3936.35,5028.39");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=4831.02,4836.72");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=5857.66,4956.17");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=840.00,4665.18");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=1335.02,4688.73");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=2565.57,4559.82");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=3786.25,4666.62");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=4479.03,4707.90");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=5475.77,4724.47");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=578.79,4398.32");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=1033.55,4347.48");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=2081.85,4472.80");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=2966.25,4368.73");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=3484.12,4422.75");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=4807.51,4369.52");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=5210.25,4329.35");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=178.33,4125.06");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=1596.21,4209.67");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=2695.74,4048.69");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=3268.21,4128.90");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=4074.12,4119.28");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=4416.26,4200.86");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=418.61,3800.39");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=761.12,3896.37");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=2383.10,3888.01");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=3739.57,3896.40");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=4374.40,3813.45");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=5766.14,3928.59");
    }

}

