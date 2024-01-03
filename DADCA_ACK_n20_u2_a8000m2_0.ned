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

network DADCA_ACK_n20_u2_a8000m2_0
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
        @display("i=misc/sensor;p=357.15,7895.73");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1045.87,7994.55");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1673.49,7981.90");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2067.14,7858.78");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2648.27,7757.23");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3076.59,7931.71");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3958.24,7791.58");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4488.95,7900.29");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5067.43,7923.40");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=197.42,7539.34");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=687.46,7512.31");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1236.09,7607.01");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1656.41,7551.09");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2954.15,7540.92");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3496.96,7706.37");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5189.08,7525.55");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1440.21,7275.24");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2043.81,7478.58");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3342.85,7388.43");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4039.47,7356.92");
    }

}

