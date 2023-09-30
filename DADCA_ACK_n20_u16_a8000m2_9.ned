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

network DADCA_ACK_n20_u16_a8000m2_9
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
        @display("i=misc/sensor;p=1097.62,7950.25");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1769.65,7978.06");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3523.63,7786.29");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4534.99,7839.67");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5185.06,7963.78");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=1265.33,7564.18");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1923.24,7646.18");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2322.83,7675.00");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3105.01,7698.62");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4112.62,7585.57");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5410.57,7665.49");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5795.85,7520.60");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=755.62,7385.98");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1486.22,7267.07");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3650.39,7339.69");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4700.61,7473.25");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5096.97,7402.99");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=385.45,7254.38");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1837.95,7118.84");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2344.57,7107.78");
    }

}

