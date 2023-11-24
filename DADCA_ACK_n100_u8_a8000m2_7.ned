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
        @display("i=misc/sensor;p=499.47,7991.79");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1827.62,7876.27");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2615.07,7975.26");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3236.57,7971.72");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4765.11,7863.46");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5657.76,7921.50");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=145.03,7632.85");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2145.97,7695.19");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3074.14,7640.73");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3775.22,7551.18");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4415.02,7731.80");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5265.86,7537.33");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=582.03,7430.91");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1024.03,7469.30");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1793.14,7405.74");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2891.40,7300.62");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4317.33,7383.49");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5884.99,7494.00");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1365.08,7134.33");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2590.90,7088.61");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=3526.37,7217.48");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=4087.76,7114.36");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4746.94,7169.93");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5289.82,7146.50");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5917.62,7125.23");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=98.37,6938.95");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=715.17,6957.13");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1164.43,6800.37");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1762.20,6923.50");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2126.71,6973.48");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2942.31,6911.03");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4485.19,6834.91");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1480.90,6616.64");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2290.86,6523.62");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3328.70,6641.80");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3772.40,6636.57");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4832.23,6739.96");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5252.47,6696.27");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5940.54,6614.89");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=225.01,6477.60");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2920.97,6452.82");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=4079.01,6451.08");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=170.54,6041.54");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=837.89,6241.42");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2451.96,6173.32");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2791.00,6039.80");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4508.39,6216.90");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=5121.47,6237.89");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5487.67,6101.29");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5891.88,6102.77");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=470.47,5818.03");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=829.45,5784.73");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=1310.34,5949.50");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=3322.97,6009.23");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=3981.67,5884.42");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=5721.17,5795.66");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=2452.24,5570.47");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=2907.95,5695.86");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=3377.93,5576.11");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=4423.01,5575.28");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=4880.41,5694.36");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=273.90,5401.38");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=1281.25,5412.39");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=1631.47,5387.55");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=2044.28,5283.49");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=5414.19,5439.42");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=1352.74,5055.42");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=2456.04,5110.63");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=2860.57,5206.33");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=3256.87,5160.70");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=3760.92,5276.16");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=4183.55,5198.94");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=4549.51,5102.23");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=5089.94,5173.45");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=5791.46,5133.24");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=721.28,4906.82");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=1856.63,4957.58");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=3015.53,4879.50");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=4312.45,4794.80");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=5407.26,4996.14");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=1341.50,4660.98");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=2367.68,4539.25");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=3909.91,4776.46");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=4824.90,4750.25");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=5680.76,4543.60");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=5969.71,4747.82");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=374.79,4435.58");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=1038.96,4467.04");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=1782.37,4512.95");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=2700.02,4365.30");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=3498.31,4433.07");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=4582.49,4453.52");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=5089.12,4387.98");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=928.63,4062.65");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=1312.91,4111.19");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=1843.73,4082.00");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=3819.43,4197.06");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=4170.73,4183.62");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=5845.40,4217.85");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=272.34,3982.79");
    }

}

