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

network DADCA_ACK_n100_a8000m2_5
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
        @display("i=misc/sensor;p=809.76,7933.51");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2604.08,7926.98");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3316.67,7984.88");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4015.04,7783.85");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4788.82,7994.78");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5268.76,7931.12");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5721.12,7889.49");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=352.03,7631.38");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1518.11,7652.64");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2724.77,7513.66");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3095.92,7665.78");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3630.41,7586.00");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5458.55,7579.71");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=847.04,7372.77");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1300.43,7285.30");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1748.73,7324.62");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2144.54,7329.60");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3268.55,7244.47");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4261.90,7297.96");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4925.17,7381.78");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=5962.57,7372.34");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=214.52,7071.23");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=686.34,7005.13");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3639.17,7050.07");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5344.29,7034.52");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1207.16,6833.76");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1732.66,6947.39");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2134.47,6793.05");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2633.71,6874.40");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3035.15,6783.91");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4039.47,6732.21");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4494.65,6718.11");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=5782.00,6950.66");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=136.73,6487.68");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3299.90,6449.01");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3592.64,6685.75");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4956.28,6566.77");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5576.96,6475.92");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5991.28,6598.16");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=525.55,6392.99");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=946.07,6385.78");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1513.42,6210.34");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2733.00,6362.35");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=4221.88,6384.61");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5955.02,6188.48");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=61.69,6028.64");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=610.75,5982.65");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1053.52,5987.91");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=2590.64,5986.10");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=3145.11,6091.69");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=3563.14,6170.85");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=4454.01,6091.19");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=5097.46,6061.80");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=2077.99,5853.41");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=3812.65,5860.81");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=4224.55,5703.15");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=5426.02,5794.39");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=191.48,5563.53");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=563.13,5609.43");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=1075.21,5564.92");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=1633.15,5545.91");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=2531.28,5575.29");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=3235.18,5577.83");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=3851.79,5490.85");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=4743.25,5601.41");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=5116.62,5517.23");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=796.14,5205.83");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=3128.03,5227.78");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=3509.71,5258.70");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=161.74,4963.36");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=1394.57,5120.57");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=1773.35,4916.82");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=2338.60,5080.21");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=2679.75,4918.92");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=4027.83,5062.35");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=4574.78,5056.06");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=5031.30,4895.28");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=5368.93,5107.83");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=613.02,4841.17");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=1260.15,4688.32");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=3261.33,4869.70");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=3691.19,4688.44");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=4416.84,4669.29");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=5747.25,4660.27");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=20.54,4535.82");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=833.59,4543.70");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=2496.84,4504.87");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=3115.70,4460.77");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=5309.22,4417.94");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=378.05,4363.16");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=1566.39,4213.56");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=2142.11,4124.24");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=3778.52,4275.42");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=4496.94,4234.36");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=4915.81,4186.97");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=5990.36,4197.74");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=1105.79,4007.62");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=2628.17,3955.73");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=3150.96,3917.50");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=353.99,3691.30");
    }

}

