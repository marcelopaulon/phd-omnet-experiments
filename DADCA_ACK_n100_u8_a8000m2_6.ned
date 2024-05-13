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

network DADCA_ACK_n100_u8_a8000m2_6
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
        @display("i=misc/sensor;p=892.03,7813.84");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1247.80,7834.94");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3358.89,7768.77");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4125.51,7852.76");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4528.86,7903.96");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5189.81,7954.35");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5973.46,7952.83");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=98.02,7591.85");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1661.01,7597.09");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2437.46,7729.10");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4951.01,7620.29");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5447.23,7676.55");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=569.03,7331.56");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1341.04,7391.56");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2752.14,7498.92");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3111.04,7405.58");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3671.52,7482.59");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4137.75,7454.07");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4480.24,7370.53");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1716.77,7148.55");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2165.37,7201.15");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2564.79,7155.20");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4231.45,7078.97");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5026.82,7081.04");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5991.46,7030.74");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=204.23,6958.34");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1163.17,6995.08");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1969.41,6808.65");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2831.06,6784.49");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3201.41,6918.53");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=3613.08,6954.16");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4536.47,6784.41");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=5545.63,6944.72");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=652.03,6675.08");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1509.39,6758.19");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2502.73,6554.35");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3419.51,6525.79");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3856.07,6649.74");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5079.99,6704.85");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5433.08,6533.51");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=186.31,6339.37");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1273.87,6468.47");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1653.39,6390.50");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2082.18,6473.39");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3000.25,6388.38");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=4305.93,6312.29");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5908.07,6408.81");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1038.94,6182.58");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=2436.27,6193.33");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=3199.51,6034.91");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=4786.90,6104.05");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=367.97,5777.04");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=717.03,5942.77");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=1349.05,5909.51");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=1856.40,5967.51");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=2567.01,5787.66");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=3588.04,5986.97");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=3963.47,5867.88");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=5381.60,5777.76");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=5723.39,5941.01");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=1052.80,5542.57");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=1812.02,5536.34");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=3412.08,5674.57");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=4461.04,5670.39");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=547.25,5367.31");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=2148.23,5396.27");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=2583.47,5336.07");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=3079.55,5509.87");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=3712.27,5415.17");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=4227.90,5408.01");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=4827.39,5475.27");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=5392.45,5407.94");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=5853.05,5409.65");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=16.31,5193.84");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=905.70,5169.63");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=1401.75,5064.23");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=2962.44,5128.88");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=4973.22,5044.94");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=541.69,4984.13");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=1900.47,4849.75");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=2410.76,4878.09");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=3440.81,4988.13");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=3833.66,4969.08");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=4223.25,5024.45");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=5568.14,4948.61");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=5992.00,4870.29");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=201.64,4609.53");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=1376.04,4666.51");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=3468.24,4559.26");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=3926.42,4553.84");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=4409.96,4627.95");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=5073.30,4658.10");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=672.26,4493.05");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=1049.46,4493.14");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=2037.46,4431.82");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=2495.06,4419.08");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=2884.96,4508.12");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=4745.11,4463.89");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=5491.63,4494.76");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=154.00,4174.33");
    }

}

