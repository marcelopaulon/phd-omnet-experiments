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
        @display("i=misc/sensor;p=1807.62,7968.57");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2705.76,7922.88");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3372.08,7788.44");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3961.71,7864.78");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5021.74,7812.82");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5598.09,7975.75");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=502.68,7749.91");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1515.65,7642.95");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2040.15,7662.83");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2633.11,7522.60");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2992.10,7623.20");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4298.39,7697.48");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5792.96,7583.27");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=273.72,7377.88");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=840.66,7492.67");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1892.51,7280.35");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3708.74,7487.37");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4042.49,7362.75");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4619.93,7277.92");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5058.08,7371.11");
    }

}

