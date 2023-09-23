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

network DADCA_ACK_n20_u4_a8000m2_8
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
        @display("i=misc/sensor;p=426.73,7780.73");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=845.32,7804.43");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1756.23,7979.69");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4461.45,7774.83");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5706.08,7769.69");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=22.12,7553.98");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1200.67,7736.92");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1658.25,7613.23");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2139.29,7710.14");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4074.78,7551.30");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4740.63,7541.74");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5285.67,7724.14");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=577.72,7364.13");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=960.84,7378.85");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2483.37,7477.25");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5183.48,7317.05");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1228.29,7065.79");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1638.41,7179.86");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3228.38,7083.59");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3837.84,7023.23");
    }

}

