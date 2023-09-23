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

network DADCA_ACK_n20_u4_a8000m2_4
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
        @display("i=misc/sensor;p=2862.05,7763.45");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=3363.13,7919.93");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3711.00,7964.85");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4117.59,7960.75");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=343.60,7584.48");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=2222.84,7747.39");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3740.91,7600.79");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4865.56,7643.60");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=37.79,7353.85");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=594.89,7286.19");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1388.45,7329.42");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1924.98,7428.93");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2648.16,7335.42");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3026.23,7370.19");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3405.48,7499.61");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4038.81,7265.79");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4505.79,7501.78");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5469.02,7489.87");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1723.26,7091.44");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2222.28,7220.48");
    }

}

