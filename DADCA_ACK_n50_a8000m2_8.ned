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

network DADCA_ACK_n50_a8000m2_8
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
        @display("i=misc/sensor;p=2076.41,7903.59");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2584.19,7791.05");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3034.80,7829.06");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4048.83,7843.42");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4501.63,7745.24");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4859.36,7872.13");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5264.72,7923.01");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=32.65,7715.27");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=675.02,7717.15");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1125.69,7642.05");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1623.09,7642.50");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3608.72,7563.51");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5631.24,7724.02");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=326.57,7357.48");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1702.82,7235.08");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2203.93,7263.50");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2605.98,7376.10");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5902.64,7232.78");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=704.40,7182.99");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1433.04,6979.18");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2902.68,7156.70");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3404.54,7059.86");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3841.41,7099.07");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4269.25,7132.53");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4829.49,7143.46");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5220.41,7200.00");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=5600.56,7013.89");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=130.60,6945.85");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4437.22,6794.26");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=812.85,6512.63");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2003.69,6597.30");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2681.31,6504.86");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=3165.34,6624.06");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3798.74,6522.18");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4790.55,6690.18");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5154.50,6497.66");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=232.06,6234.58");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1527.01,6372.53");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=3296.42,6261.73");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4446.74,6347.34");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5740.29,6431.49");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=882.59,6095.82");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2379.54,6186.46");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=4164.18,6052.99");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=306.96,5696.06");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1712.39,5869.08");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2165.46,5748.69");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=2668.09,5806.43");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3121.92,5682.79");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=3624.20,5899.96");
    }

}

