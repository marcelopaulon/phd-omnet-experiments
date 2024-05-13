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

network DADCA_ACK_n20_u8_a8000m2_6
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
        @display("i=misc/sensor;p=660.85,7795.00");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1316.67,7823.69");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2163.96,7905.40");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3877.03,7870.30");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4572.23,7926.93");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5004.22,7946.68");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=2774.54,7685.01");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3513.47,7670.00");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4768.08,7597.05");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5563.04,7563.31");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5925.61,7700.34");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=335.48,7310.92");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=684.39,7422.72");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1108.33,7398.01");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2496.26,7346.86");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3187.76,7298.62");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4398.63,7349.71");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5047.18,7355.05");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5916.43,7309.05");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1926.92,7237.44");
    }

}

