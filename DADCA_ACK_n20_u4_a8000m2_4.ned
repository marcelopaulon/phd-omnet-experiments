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
        @display("i=misc/sensor;p=910.54,7819.03");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1517.07,7777.19");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1942.25,7796.75");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2548.60,7758.95");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2969.01,7896.12");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4378.95,7775.02");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4915.68,7905.34");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5321.24,7803.29");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5970.06,7944.91");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4.83,7600.04");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=645.14,7548.01");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3294.01,7678.10");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4040.80,7544.56");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1165.43,7380.08");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1556.68,7415.89");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4670.17,7388.84");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5073.90,7279.42");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5487.12,7319.34");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5941.44,7389.84");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=21.86,7093.63");
    }

}

