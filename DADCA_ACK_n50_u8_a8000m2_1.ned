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

network DADCA_ACK_n50_u8_a8000m2_1
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
        @display("i=misc/sensor;p=257.67,7861.63");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1991.28,7954.89");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2304.84,7793.61");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2837.26,7772.29");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4369.31,7929.70");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4965.99,7930.24");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5639.78,7987.28");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=649.96,7722.23");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1420.61,7715.87");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3932.38,7701.35");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4554.32,7516.07");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5754.81,7542.21");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=218.78,7489.05");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1800.30,7345.73");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2160.77,7418.97");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2597.95,7282.98");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3062.57,7263.55");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3420.45,7283.26");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3812.48,7352.26");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5140.46,7401.17");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=30.31,7134.40");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=573.37,7242.68");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1054.70,7207.12");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4302.14,7229.01");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=429.67,6910.93");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1950.58,6907.81");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2522.91,6861.70");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3830.83,6866.68");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4205.44,6869.95");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4588.97,6880.14");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5045.47,6865.33");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=5439.94,6901.16");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=112.39,6699.14");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=774.60,6755.12");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1384.19,6610.23");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2051.55,6555.11");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3128.16,6747.43");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5606.94,6555.18");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=321.52,6385.00");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1129.28,6307.45");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2743.19,6511.38");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3379.16,6451.80");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4135.26,6380.52");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=4588.96,6405.68");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4965.70,6292.43");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5316.82,6304.16");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=753.24,6085.63");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=2890.96,6188.70");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3287.18,6071.67");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=3806.47,6072.01");
    }

}

