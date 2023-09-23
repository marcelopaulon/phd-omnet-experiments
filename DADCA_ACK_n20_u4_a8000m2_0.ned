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

network DADCA_ACK_n20_u4_a8000m2_0
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
        @display("i=misc/sensor;p=765.86,7993.19");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1239.79,7876.41");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3351.57,7939.44");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4482.83,7917.40");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4890.51,7980.95");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5328.18,7946.71");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5992.92,7843.32");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=177.10,7632.29");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=648.64,7624.37");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1735.76,7531.41");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2273.98,7532.15");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2646.02,7537.70");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3940.18,7652.64");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4545.70,7529.20");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4939.38,7623.78");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=981.93,7269.97");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5703.67,7399.78");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=137.48,7016.98");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1806.09,7160.69");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2184.22,7053.99");
    }

}

