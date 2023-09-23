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

network DADCA_ACK_n50_u4_a8000m2_0
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
        @display("i=misc/sensor;p=450.84,7991.67");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=955.96,7941.02");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2179.38,7861.28");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2948.36,7964.68");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3628.27,7942.35");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4141.62,7798.32");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4723.17,7780.79");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5348.50,7987.47");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5847.50,7860.69");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1794.87,7744.24");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2668.09,7594.91");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3311.88,7604.55");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=216.32,7327.98");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=723.48,7490.90");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1333.15,7502.06");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3504.57,7265.91");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3929.34,7401.91");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4613.55,7412.51");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5246.98,7376.72");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=852.61,7018.17");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1413.45,7143.85");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1907.02,7193.52");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=2426.97,7145.45");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5858.88,7250.92");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=300.56,6944.19");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2815.17,6959.03");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3263.97,6823.81");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3802.93,6968.41");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4221.91,6861.45");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4623.20,7003.17");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5093.27,6939.87");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=5886.31,6871.05");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=509.35,6536.11");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1075.26,6729.72");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2021.09,6733.93");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2388.16,6581.20");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2745.91,6556.62");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4532.51,6654.88");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5525.77,6761.19");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=938.52,6361.75");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1662.71,6473.10");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=4035.70,6467.12");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4854.14,6433.70");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5227.29,6388.79");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=82.15,6250.45");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1497.27,6114.77");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2263.05,6227.05");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3391.18,6204.45");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5483.29,6134.62");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=549.98,6013.52");
    }

}

