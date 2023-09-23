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

network DADCA_ACK_n50_u8_a8000m2_4
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
        @display("i=misc/sensor;p=786.54,7816.84");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1223.27,7878.00");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1711.57,7961.78");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2233.52,7943.26");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3788.93,7880.30");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4244.68,7985.64");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5322.92,7818.11");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5780.58,7848.26");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=997.75,7508.00");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2252.23,7560.38");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2721.80,7551.81");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3251.98,7666.75");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4125.97,7545.26");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4811.77,7582.61");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=604.97,7478.22");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1500.96,7396.27");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1851.58,7357.77");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3329.03,7281.65");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3806.24,7305.73");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5392.98,7451.89");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=5856.66,7424.14");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2847.99,7118.30");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4413.40,7256.63");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5026.60,7238.13");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=280.36,6859.19");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=782.84,6978.03");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1529.06,6965.81");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3308.81,6827.45");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3783.16,6936.32");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4233.40,6798.15");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2000.25,6695.64");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2390.09,6752.37");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=3047.72,6522.77");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=4705.50,6558.36");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5183.97,6662.91");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5676.40,6554.73");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=749.72,6504.89");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1415.11,6336.34");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1971.63,6306.84");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2385.67,6394.07");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4019.69,6457.80");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5181.87,6283.74");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=345.68,6222.53");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=723.45,6085.84");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2794.82,6042.17");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3658.13,6117.72");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4522.24,6200.68");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=5889.50,6062.27");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=384.88,5818.19");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1087.45,5992.33");
    }

}

