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

network DADCA_ACK_n20_u2_a8000m2_9
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
        @display("i=misc/sensor;p=93.29,7853.96");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=778.26,7844.71");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2518.24,7998.85");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3116.09,7873.12");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3869.56,7900.77");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4322.68,7939.99");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4778.74,7772.90");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1120.09,7700.10");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1584.56,7614.05");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2784.05,7533.49");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4507.29,7533.07");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5211.01,7664.66");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5728.81,7661.08");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=34.98,7283.00");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=378.20,7396.40");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=952.05,7297.41");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1933.29,7412.97");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2374.63,7400.17");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3521.82,7439.87");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4210.15,7337.39");
    }

}

