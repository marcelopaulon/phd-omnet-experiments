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

network DADCA_ACK_n20_u3_a8000m2_9
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
        @display("i=misc/sensor;p=347.33,7870.90");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=3573.88,7814.38");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=4149.98,7817.01");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4555.36,7995.96");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5252.78,7953.27");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5930.91,7896.01");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1268.22,7716.64");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1727.95,7542.27");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5608.64,7551.71");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=768.32,7464.29");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1414.69,7316.58");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3051.62,7431.10");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3742.80,7498.92");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4329.11,7502.18");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4852.43,7272.42");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5986.92,7502.19");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1010.09,7178.03");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2159.82,7215.44");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2684.75,7205.95");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3360.69,7142.80");
    }

}

