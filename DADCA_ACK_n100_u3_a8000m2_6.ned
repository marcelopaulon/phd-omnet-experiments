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

network DADCA_ACK_n100_u3_a8000m2_6
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
        @display("i=misc/sensor;p=307.50,7931.04");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=673.63,7798.97");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1216.19,7979.69");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=1762.12,7922.24");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2160.34,7899.74");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=2554.78,7781.42");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3240.59,7980.14");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4179.22,7844.34");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=44.97,7559.35");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=984.36,7578.66");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1434.26,7660.37");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1883.48,7541.74");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3602.76,7739.04");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4709.57,7735.85");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2382.43,7384.14");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3383.51,7322.74");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3788.79,7360.47");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4155.50,7484.67");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4551.67,7313.24");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5122.99,7313.51");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=585.56,7250.28");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1040.67,7118.17");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1718.52,7073.18");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=2080.68,7053.82");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5690.00,7182.78");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=83.53,6987.38");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2955.79,6936.08");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3490.10,6806.59");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3927.47,6982.04");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4975.78,6930.68");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5357.61,6946.06");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=5930.29,6794.71");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=909.68,6679.20");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1856.16,6738.32");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2437.26,6708.56");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4128.43,6632.51");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4482.65,6709.15");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5397.84,6595.19");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=179.02,6350.41");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=604.80,6375.27");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1344.94,6508.53");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1958.31,6394.00");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2747.19,6433.81");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3284.86,6485.86");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3914.90,6293.76");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=4791.47,6429.42");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5810.90,6384.57");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=888.79,6087.62");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1495.86,6102.83");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=2218.24,6082.90");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=3506.39,6111.09");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=4371.84,6055.64");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=5016.45,6046.83");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=5434.81,6138.13");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=301.08,5773.84");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=1147.00,5847.85");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=3127.71,5871.17");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=3829.19,5927.51");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=2071.97,5745.92");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=2770.73,5585.69");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=3538.39,5547.51");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=4219.70,5652.38");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=4768.48,5569.42");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=5721.28,5698.21");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=736.66,5286.36");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=1074.39,5453.64");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=1599.37,5473.84");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=2320.16,5399.64");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=4424.58,5320.93");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=5286.75,5512.57");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=153.05,5227.72");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=1265.17,5052.84");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=3015.87,5184.04");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=3504.43,5043.04");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=4797.75,5084.47");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=5647.19,5268.15");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=486.30,4924.96");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=2019.63,4960.99");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=3998.71,5022.72");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=5835.36,4847.82");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=225.81,4535.58");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=935.56,4753.34");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=1371.27,4538.02");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=1796.57,4684.09");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=2422.25,4599.00");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=3042.12,4778.91");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=3409.59,4570.54");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=4165.31,4591.94");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=4543.83,4750.72");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=5207.03,4758.56");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=730.58,4446.81");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=1936.21,4349.12");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=4473.92,4402.54");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=4860.03,4496.15");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=5471.25,4406.78");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=321.36,4085.14");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=1149.52,4248.49");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=1601.29,4185.79");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=2251.43,4099.92");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=2704.60,4229.56");
    }

}

