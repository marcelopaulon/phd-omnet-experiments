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

network DADCA_ACK_n50_u3_a8000m2_9
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
        @display("i=misc/sensor;p=1234.13,7993.19");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1642.36,7815.43");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3250.72,7904.22");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3750.27,7763.00");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4338.72,7832.33");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4717.87,7918.02");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=41.11,7576.91");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=550.17,7589.40");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1324.39,7534.33");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1990.54,7666.07");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2729.74,7551.73");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5918.57,7567.27");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=973.65,7503.41");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3490.21,7388.02");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4308.21,7406.34");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4890.38,7448.14");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5571.14,7304.92");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=90.46,7119.78");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1310.52,7033.49");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1910.64,7077.25");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2783.04,7087.03");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3192.77,7059.61");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3885.62,7081.34");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=459.62,6930.58");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2415.76,6844.83");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=3576.43,6897.39");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=4349.65,6896.15");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4710.70,6822.13");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5221.86,6923.72");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=173.10,6532.18");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=818.90,6563.00");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1836.83,6620.14");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2955.55,6707.49");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1343.73,6372.82");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2431.01,6431.33");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2885.37,6349.88");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3304.11,6479.99");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3771.61,6513.93");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4625.83,6465.87");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5043.37,6492.56");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5430.63,6306.78");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=163.47,6124.64");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1905.94,6190.89");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2686.79,6047.61");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4985.84,6062.25");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=409.35,5790.08");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=998.80,5997.14");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1425.58,5988.65");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3499.45,5921.46");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4128.00,5990.67");
    }

}

