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

network DADCA_ACK_n20_u3_a8000m2_2
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
        @display("i=misc/sensor;p=548.38,7835.68");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1147.33,7903.83");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1500.37,7811.71");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3221.71,7855.86");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=840.54,7605.05");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=1235.37,7527.30");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=2028.52,7708.56");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2629.13,7570.92");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3757.33,7725.31");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4053.55,7533.01");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4585.82,7563.74");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5157.04,7669.76");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1685.40,7381.20");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3229.87,7401.77");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3606.06,7383.38");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=454.57,7083.45");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1083.07,7203.15");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2331.46,7205.48");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4311.51,7177.07");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4697.73,7158.52");
    }

}

