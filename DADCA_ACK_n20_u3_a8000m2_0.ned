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

network DADCA_ACK_n20_u3_a8000m2_0
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
        @display("i=misc/sensor;p=480.87,7779.26");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1137.83,7859.06");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1474.26,7758.43");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=1968.49,7860.41");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3566.10,7904.37");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3962.89,7826.47");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4592.75,7758.27");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5422.67,7842.90");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5793.03,7984.16");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=80.52,7575.34");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=852.01,7643.48");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2530.80,7722.16");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3543.54,7538.03");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=669.98,7335.66");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1357.30,7399.13");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1809.25,7463.28");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2435.14,7356.58");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3098.47,7431.98");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4048.83,7449.50");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4892.00,7415.29");
    }

}

